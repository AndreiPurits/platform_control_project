# -*- coding: utf-8 -*-  # roadseg.py
import os
import numpy as np


class RoadSeg:
    """
    Обёртка над ONNX-моделью сегментации дороги.

    Логика:
      - если onnxruntime не установлен,
      - или путь к модели не задан / файл не найден,
      - или не удалось создать InferenceSession,
    то self.ok = False, а infer(...) всегда возвращает НУЛЕВУЮ маску
    (никакой зелени, никакой «дороги»).

    Масштаб входа (input_size) автоматически считывается из модели:
      (N, C, H, W) -> input_size = (W, H).
    """

    def __init__(self, onnx_path: str | None = None, input_size: tuple[int, int] | None = None):
        # input_size храним как (W, H)
        self.input_size: tuple[int, int] = (384, 384)  # дефолт, потом перезапишем из модели
        if input_size is not None:
            self.input_size = tuple(input_size)

        self.ok: bool = False
        self.session = None
        self.iname = None
        self.oname = None
        self._rt = None  # модуль onnxruntime

        # --- 1. Если путь к модели не задан — сразу stub (всё = 0) ---
        if not onnx_path:
            print("[SEG] no onnx_path, segmentation stub (zeros mask)", flush=True)
            return

        # --- 2. Импорт onnxruntime ---
        try:
            import onnxruntime as ort
            self._rt = ort
        except Exception as e:
            print("[SEG] onnxruntime not available, segmentation stub (zeros):", e, flush=True)
            return

        # --- 3. Проверка наличия файла модели ---
        if not os.path.isfile(onnx_path):
            print(f"[SEG] ONNX file not found, segmentation stub (zeros): {onnx_path}", flush=True)
            return

        # --- 4. Создаём InferenceSession и читаем размер входа ---
        try:
            sess = self._rt.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
            self.session = sess

            inp = sess.get_inputs()[0]
            out = sess.get_outputs()[0]
            self.iname = inp.name
            self.oname = out.name

            # shape ожидаем [N, C, H, W]
            shape = inp.shape
            if (
                isinstance(shape, (list, tuple))
                and len(shape) == 4
                and shape[2] not in (None, "None")
                and shape[3] not in (None, "None")
            ):
                H_model = int(shape[2])
                W_model = int(shape[3])
                self.input_size = (W_model, H_model)

            self.ok = True
            print(f"[SEG] ONNX loaded: {onnx_path}  input_size={self.input_size}", flush=True)

        except Exception as e:
            print("[SEG] failed to create InferenceSession, segmentation stub (zeros):", e, flush=True)
            self.session = None
            self.ok = False

    # ------------------------------------------------------------------
    #   ИНФЕРЕНС
    # ------------------------------------------------------------------
    def infer(self, bgr_np: np.ndarray) -> np.ndarray:
        """
        bgr_np: HxWx3 uint8 (OpenCV кадр BGR)
        return: HxW float32 маска в [0..1]

        Если self.ok == False → возвращаем НУЛЕВУЮ маску (дороги нет).
        """

        import cv2

        # sanity-check входа
        if bgr_np is None or bgr_np.ndim != 3 or bgr_np.shape[2] != 3:
            return np.zeros((1, 1), np.float32)

        H_in, W_in = bgr_np.shape[:2]

        # --- если модель не готова — вообще НЕТ дороги ---
        if (not self.ok) or (self.session is None):
            return np.zeros((H_in, W_in), np.float32)

        W_model, H_model = self.input_size  # (W, H)

        # ресайз под модель
        small = cv2.resize(
            bgr_np,
            (W_model, H_model),
            interpolation=cv2.INTER_LINEAR
        )

        # подготовка входа: BGR → RGB, [0..1], NCHW
        x = small[:, :, ::-1].astype(np.float32) / 255.0  # BGR->RGB
        x = np.transpose(x, (2, 0, 1))[None, ...]         # (1,3,H,W)
        try:
            out = self.session.run([self.oname], {self.iname: x})[0]
        except Exception as e:
            # на ошибке инференса — лог и нулевая маска
            print("[SEG] runtime error, returning zeros:", e, flush=True)
            return np.zeros((H_in, W_in), np.float32)

        # допускаем [N,1,H,W] или [N,H,W]
        if out.ndim == 4:
            out = out[:, 0, :, :]
        out = out[0]  # (H,W)

        # сигмоида — перевод логитов в вероятности
        out = 1.0 / (1.0 + np.exp(-out))

        mask_small = out.astype(np.float32)

        # вернём маску в исходный размер кадра
        mask = cv2.resize(
            mask_small,
            (W_in, H_in),
            interpolation=cv2.INTER_LINEAR
        )
        return mask
    
import numpy as np

def detect_junction_in_mask(mask: np.ndarray,
                            band_rel=(0.3, 0.6),
                            thr: float = 0.5,
                            min_center_frac: float = 0.15,
                            min_side_frac: float = 0.08) -> dict:
    """
    mask: HxW float32 [0..1] — маска дороги.
    
    Ищем "пояс" в верхней/средней части кадра и проверяем,
    что дорога есть по центру и хотя бы с одной стороны (лево/право).
    
    Возвращает:
      {
        "is_junction": bool,
        "left":  bool,   # есть дорога слева
        "right": bool    # есть дорога справа
      }
    """
    if mask is None or mask.ndim != 2:
        return {"is_junction": False, "left": False, "right": False}

    H, W = mask.shape
    y0 = int(H * band_rel[0])
    y1 = int(H * band_rel[1])
    y0 = max(0, min(H-1, y0))
    y1 = max(y0+1, min(H,   y1))

    band = mask[y0:y1, :]
    if band.size == 0:
        return {"is_junction": False, "left": False, "right": False}

    road = (band >= thr)
    if not road.any():
        return {"is_junction": False, "left": False, "right": False}

    w_third = W // 3
    if w_third <= 0:
        return {"is_junction": False, "left": False, "right": False}

    left_zone   = road[:, :w_third]
    center_zone = road[:, w_third:2*w_third]
    right_zone  = road[:, 2*w_third:]

    total = float(road.size)
    left_frac   = left_zone.sum()   / total
    center_frac = center_zone.sum() / total
    right_frac  = right_zone.sum()  / total

    has_center = center_frac >= min_center_frac
    has_left   = left_frac   >= min_side_frac
    has_right  = right_frac  >= min_side_frac

    is_junction = has_center and (has_left or has_right)

    return {
        "is_junction": bool(is_junction),
        "left":  bool(has_left),
        "right": bool(has_right),
    }

from routing import nearest_junction_ahead
from roadseg import detect_junction_in_mask

def maybe_snap_robot_to_junction_by_camera(state,
                                           mask,
                                           max_graph_dist_m: float = 5.0):
    """
    Если по графу впереди есть перекрёсток ближе max_graph_dist_m
    и в маске видно развилку, считаем, что мы "на" перекрёстке
    и подщёлкиваем robot_px к координатам этого перекрёстка.
    """
    res = nearest_junction_ahead(state, max_dist_m=max_graph_dist_m)
    if not res:
        return False

    dist_m, jpx = res      # (dist_to_junction_m, (jx,jy))

    info = detect_junction_in_mask(mask)
    if not info.get("is_junction", False):
        return False

    # СНАП!
    state.robot_px = (float(jpx[0]), float(jpx[1]))
    # обновим трек позиции, чтобы все FSM/логика разворота не сходили с ума
    try:
        from robot_cmd import note_robot_pose  # если в этом же файле — не нужно
        note_robot_pose(state)
    except Exception:
        pass

    print(f"[SNAP CAM] snapped robot to junction {jpx} (dist_graph≈{dist_m:.1f} м)", flush=True)
    return True