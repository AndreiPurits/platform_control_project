# -*- coding: utf-8 -*-
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


class AutoDatasetRecorder:
    """
    Автосохранение по шагу route_done_m.

    Папка: ~/datasets/photos/<YYYYmmdd_HHMMSS>
    На каждую запись (РОВНО 2 файла):
      - img_000123_done_45.0m_ts_1700000000.jpg
      - mask_000123_done_45.0m_ts_1700000000.png   (road_bin -> 0/255)
    """

    def __init__(
        self,
        step_m: float = 3.0,
        base_dir: Optional[str] = None,
        img_ext: str = ".jpg",
        jpg_quality: int = 92,
        use_max_done: bool = True,
        save_mask: bool = False,   # сырой seg НЕ сохраняем
        save_bin: bool = True,     # бинарку сохраняем как mask_*.png
        mask_ext: str = ".png",
    ):
        self.step_m = float(step_m)
        self.img_ext = img_ext.lower()
        self.jpg_quality = int(jpg_quality)
        self.use_max_done = bool(use_max_done)

        self.save_mask = bool(save_mask)
        self.save_bin = bool(save_bin)
        self.mask_ext = mask_ext.lower()

        if base_dir is None:
            base_dir = str(Path.home() / "datasets" / "photos")
        self.base_dir = Path(base_dir).expanduser()

        self.session_dir: Optional[Path] = None
        self._frame_idx = 0
        self._last_saved_done = None
        self._max_seen_done = 0.0

        self._last_frame_bgr: Optional[np.ndarray] = None
        self._last_seg_mask: Optional[np.ndarray] = None
        self._last_road_bin: Optional[np.ndarray] = None

    def start_new_session(self) -> Path:
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.base_dir / ts
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self._frame_idx = 0
        self._last_saved_done = None
        self._max_seen_done = 0.0
        #print(f"[AUTO DATASET] session_dir = {self.session_dir}", flush=True)
        return self.session_dir

    def set_last_frame(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr is None or getattr(frame_bgr, "size", 0) == 0:
            return
        self._last_frame_bgr = frame_bgr

    def set_last_masks(self, seg_mask=None, road_bin=None) -> None:
        if seg_mask is not None and getattr(seg_mask, "size", 0) > 0:
            self._last_seg_mask = seg_mask
        if road_bin is not None and getattr(road_bin, "size", 0) > 0:
            self._last_road_bin = road_bin

    def update(self, state) -> None:
        if self.session_dir is None:
            self.start_new_session()

        done = getattr(state, "route_done_m", None)
        if done is None:
            return
        try:
            done = float(done)
        except Exception:
            return

        if self.use_max_done:
            if done > self._max_seen_done:
                self._max_seen_done = done
            done_eff = self._max_seen_done
        else:
            done_eff = done

        # первое сохранение
        if self._last_saved_done is None:
            if self._last_frame_bgr is not None:
                self._save_pair(done_eff)
                self._last_saved_done = done_eff
            return

        # шаг
        if (done_eff - self._last_saved_done) >= self.step_m:
            if self._last_frame_bgr is None:
                return
            while (done_eff - self._last_saved_done) >= self.step_m:
                self._last_saved_done += self.step_m
                self._save_pair(self._last_saved_done)

    def _save_pair(self, done_m: float) -> None:
        if self._last_frame_bgr is None or self.session_dir is None:
            return

        self._frame_idx += 1
        ts = int(time.time())

        # ---- image ----
        img_name = f"img_{self._frame_idx:06d}_done_{done_m:.1f}m_ts_{ts}{self.img_ext}"
        img_path = self.session_dir / img_name

        img = self._last_frame_bgr
        if self.img_ext in (".jpg", ".jpeg"):
            cv2.imwrite(str(img_path), img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_quality])
        else:
            cv2.imwrite(str(img_path), img)

        # ---- бинарная маска дороги -> mask_*.png ----
        if self.save_bin and self._last_road_bin is not None:
            b = self._last_road_bin
            if b.ndim == 3:
                b = b[:, :, 0]
            bb = (b.astype(np.uint8) > 0).astype(np.uint8) * 255

            mask_name = f"mask_{self._frame_idx:06d}_done_{done_m:.1f}m_ts_{ts}{self.mask_ext}"
            mask_path = self.session_dir / mask_name
            cv2.imwrite(str(mask_path), bb)

        # (опционально: сырой seg_mask оставили в коде, но по умолчанию выключен)
        if self.save_mask and self._last_seg_mask is not None:
            m = self._last_seg_mask
            if m.ndim == 3:
                m = m[:, :, 0]
            mm = self._to_u8_01(m)
            raw_name = f"seg_{self._frame_idx:06d}_done_{done_m:.1f}m_ts_{ts}{self.mask_ext}"
            raw_path = self.session_dir / raw_name
            cv2.imwrite(str(raw_path), mm)

        #print(f"[AUTO DATASET] saved {img_name}", flush=True)

    @staticmethod
    def _to_u8_01(x: np.ndarray) -> np.ndarray:
        xf = x.astype(np.float32)
        xf = np.nan_to_num(xf, nan=0.0, posinf=1.0, neginf=0.0)
        xf = np.clip(xf, 0.0, 1.0)
        return (xf * 255.0 + 0.5).astype(np.uint8)


class DatasetController:
    def __init__(self, state, step_m: float = 3.0, base_dir: Optional[str] = None):
        self.state = state
        self.rec = AutoDatasetRecorder(
            step_m=step_m,
            base_dir=base_dir,
            save_mask=False,   # ВАЖНО: ровно 2 файла
            save_bin=True,
        )
        self.enabled = True

    def on_start(self) -> None:
        if not self.enabled:
            return
        try:
            self.rec.start_new_session()
        except Exception:
            pass

    def on_stop(self) -> None:
        pass

    def on_progress_tick(self) -> None:
        if not self.enabled:
            return
        try:
            self.rec.update(self.state)
        except Exception:
            pass

    def maybe_capture(self, frame_bgr=None, seg_mask=None, road_bin=None, source: str = "cam", **kwargs):
        if not self.enabled:
            return

        if frame_bgr is not None:
            try:
                self.rec.set_last_frame(frame_bgr)
            except Exception:
                pass

        try:
            self.rec.set_last_masks(seg_mask=seg_mask, road_bin=road_bin)
        except Exception:
            pass

        # триггерим сохранение по done_m
        try:
            self.rec.update(self.state)
        except Exception:
            pass