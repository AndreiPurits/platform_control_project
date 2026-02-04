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

        Модель всегда работает по ВЕСЬ кадр.
        Обрезку по вертикали (ROI) делаем уже в road_follow.
        """
        import cv2

        # sanity-check
        if bgr_np is None or bgr_np.ndim != 3 or bgr_np.shape[2] != 3:
            return np.zeros((1, 1), np.float32)

        H_in, W_in = bgr_np.shape[:2]

        # если модель не готова — нулевая маска
        if (not self.ok) or (self.session is None):
            return np.zeros((H_in, W_in), np.float32)

        W_model, H_model = self.input_size  # (W, H)

        # ресайз под модель (без обрезки)
        small = cv2.resize(
            bgr_np,
            (W_model, H_model),
            interpolation=cv2.INTER_LINEAR,
        )

        # подготовка входа: BGR → RGB, [0..1], NCHW
        x = small[:, :, ::-1].astype(np.float32) / 255.0  # BGR->RGB
        x = np.transpose(x, (2, 0, 1))[None, ...]         # (1,3,H,W)

        try:
            out = self.session.run([self.oname], {self.iname: x})[0]
        except Exception as e:
            print("[SEG] runtime error, returning zeros:", e, flush=True)
            return np.zeros((H_in, W_in), np.float32)

        # допускаем [N,1,H,W] или [N,H,W]
        if out.ndim == 4:
            out = out[:, 0, :, :]
        out = out[0]  # (H,W)

        # сигмоида → вероятности
        out = 1.0 / (1.0 + np.exp(-out))
        mask_small = out.astype(np.float32)

        # возвращаем в размер исходного кадра
        mask = cv2.resize(
            mask_small,
            (W_in, H_in),
            interpolation=cv2.INTER_LINEAR,
        )
        return mask

    
