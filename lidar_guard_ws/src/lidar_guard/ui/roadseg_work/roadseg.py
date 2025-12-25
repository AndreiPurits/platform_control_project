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
    то self.ok = False, а infer(...) всегда возвращает None
    (и камера уйдёт в no_road).

    Масштаб входа (input_size) автоматически считывается из модели:
      (N, C, H, W) -> input_size = (W, H).
    """

    def __init__(
        self,
        onnx_path: str | None = None,
        input_size: tuple[int, int] | None = None,
        providers: list[str] | None = None,
        verbose: bool = True,
    ):
        # input_size храним как (W, H)
        self.input_size: tuple[int, int] = (384, 384)  # дефолт, потом перезапишем из модели
        if input_size is not None:
            self.input_size = tuple(input_size)

        self.ok: bool = False
        self.session = None
        self.iname = None
        self.oname = None
        self._rt = None  # модуль onnxruntime
        self._verbose = bool(verbose)

        # --- 1) Если путь к модели не задан — сразу stub ---
        if not onnx_path:
            if self._verbose:
                print("[SEG] no onnx_path, segmentation stub (mask=None)", flush=True)
            return

        # --- 2) Импорт onnxruntime ---
        try:
            import onnxruntime as ort
            self._rt = ort
        except Exception as e:
            if self._verbose:
                print("[SEG] onnxruntime not available, segmentation stub (mask=None):", e, flush=True)
            return

        # --- 3) Проверка файла ---
        if not os.path.isfile(onnx_path):
            if self._verbose:
                print(f"[SEG] ONNX file not found, segmentation stub (mask=None): {onnx_path}", flush=True)
            return

        # --- 4) Провайдеры ---
        if providers is None:
            providers = ["CPUExecutionProvider"]

        # --- helpers ---
        def _try_int_dim(d):
            """Безопасно превращаем dim в int, если это реально число."""
            try:
                # None / 'None' / 'height' / 'width' и т.п. -> не трогаем
                if d is None:
                    return None
                if isinstance(d, str):
                    return None
                # np.int64 и т.п.
                if isinstance(d, (int, np.integer)):
                    return int(d)
                # иногда бывает float (редко)
                if isinstance(d, float):
                    return int(d)
            except Exception:
                return None
            return None

        # --- 5) InferenceSession + читаем input shape ---
        try:
            sess = self._rt.InferenceSession(onnx_path, providers=providers)
            self.session = sess

            inputs = sess.get_inputs()
            outputs = sess.get_outputs()

            if not inputs or not outputs:
                raise RuntimeError("ONNX has no inputs/outputs")

            inp = inputs[0]
            out = outputs[0]
            self.iname = inp.name
            self.oname = out.name

            # shape ожидаем [N, C, H, W], но могут быть None/строки
            shape = inp.shape
            H_model = None
            W_model = None
            if isinstance(shape, (list, tuple)) and len(shape) == 4:
                H_model = _try_int_dim(shape[2])
                W_model = _try_int_dim(shape[3])

            if H_model is not None and W_model is not None and H_model > 0 and W_model > 0:
                self.input_size = (int(W_model), int(H_model))

            self.ok = True

            if self._verbose:
                # один раз полезный лог
                print(f"[SEG] ONNX loaded: {onnx_path}", flush=True)
                print(f"[SEG] providers={providers}", flush=True)
                for i, ii in enumerate(inputs):
                    print(f"[SEG] input[{i}] name={ii.name} shape={ii.shape} type={ii.type}", flush=True)
                for i, oo in enumerate(outputs):
                    print(f"[SEG] output[{i}] name={oo.name} shape={oo.shape} type={oo.type}", flush=True)
                print(f"[SEG] using input_size={self.input_size}", flush=True)

        except Exception as e:
            if self._verbose:
                print("[SEG] failed to create InferenceSession, segmentation stub (mask=None):", e, flush=True)
            self.session = None
            self.ok = False

    # ------------------------------------------------------------------
    #   ИНФЕРЕНС
    # ------------------------------------------------------------------
    def infer(self, bgr_np: np.ndarray):
        """
        bgr_np: HxWx3 uint8 (OpenCV кадр BGR)
        return: HxW float32 маска в [0..1] ИЛИ None (если seg не готов/ошибка)

        Модель всегда работает по ВЕСЬ кадр.
        Обрезку по вертикали (ROI) делаем уже в road_follow.
        """
        import cv2

        # sanity-check
        if bgr_np is None or not isinstance(bgr_np, np.ndarray) or bgr_np.ndim != 3 or bgr_np.shape[2] != 3:
            return None

        H_in, W_in = bgr_np.shape[:2]
        if H_in < 2 or W_in < 2:
            return None

        # если модель не готова — stub
        if (not self.ok) or (self.session is None):
            return None

        W_model, H_model = self.input_size  # (W, H)

        # ресайз под модель (без обрезки)
        small = cv2.resize(
            bgr_np,
            (int(W_model), int(H_model)),
            interpolation=cv2.INTER_LINEAR,
        )

        # подготовка входа: BGR → RGB, [0..1], NCHW
        x = small[:, :, ::-1].astype(np.float32) / 255.0  # BGR->RGB
        x = np.transpose(x, (2, 0, 1))[None, ...]         # (1,3,H,W)

        try:
            out = self.session.run([self.oname], {self.iname: x})[0]
        except Exception as e:
            if self._verbose:
                print("[SEG] runtime error, returning None:", e, flush=True)
            return None

        # допускаем [N,1,H,W] или [N,H,W]
        if out is None:
            return None

        out = np.asarray(out)
        if out.ndim == 4:
            out = out[:, 0, :, :]
        if out.ndim != 3:
            return None

        out = out[0]  # (H,W)

        # сигмоида → вероятности (с clip против переполнения exp)
        out = np.clip(out, -50.0, 50.0)
        out = 1.0 / (1.0 + np.exp(-out))
        mask_small = out.astype(np.float32)

        # возвращаем в размер исходного кадра
        mask = cv2.resize(
            mask_small,
            (int(W_in), int(H_in)),
            interpolation=cv2.INTER_LINEAR,
        )
        return mask