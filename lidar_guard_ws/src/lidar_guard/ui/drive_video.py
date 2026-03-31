# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import sys
import time
import glob
from typing import Optional, Tuple
import math
import numpy as np
import cv2
from PyQt5 import QtCore, QtWidgets, QtGui

import multiprocessing as mp
from multiprocessing.connection import Connection
from routing import (
    update_junction_turn_hint,
    maybe_snap_robot_to_junction_by_camera,
    detect_turn_branches_from_roadbin,
    update_cam_lr_debounced,
)

from drive_road_follow import RoadFollowController
from robot_cmd import motors_set

from drive_rails import (
    compute_main_road_from_roadbin,
    draw_main_road_overlay,
    draw_rails_overlay,
    update_rail_anchors_from_roadbin,
)

# ============================================================
# modes (UI panels already use these names)
# ============================================================
MODE_MARKERS = "markers"
MODE_ROAD    = "road"
MODE_TRAJ    = "traj"


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class VideoController(QtCore.QObject):
    """
    Camera orchestrator:
      - MODE_MARKERS: 4K camera + ArUco worker + marker drive (if running & not manual & not traj)
      - MODE_ROAD:    1280x720 + RoadSeg + road_bin + road_follow + (optional rails overlays)
      - MODE_TRAJ:    camera can still render/overlay, but no auto motor control here (trajectory controller does)

    IMPORTANT:
      - Manual drive must be available always => if state.manual_drive True: never call motors_set here.
      - ROADSEGDIR is provided by state.road_model_path now.
      - rails are enabled by state.rails_enabled (bool).
    """

    def __init__(
        self,
        ui: QtWidgets.QMainWindow,
        state,
        radar_scene_provider=None,
        dataset=None,
        radar_view: Optional[QtWidgets.QGraphicsView] = None,
    ):
        super().__init__(ui)
        self.ui = ui
        self.state = state
        self.dataset = dataset
        self._radar_scene_provider = radar_scene_provider
        self._radar_view = radar_view
        # --- telemetry labels (Qt UI) ---
        self._lblTelemRow1 = self.ui.findChild(QtWidgets.QLabel, "lblTelemRow1")
        self._lblTelemRow2 = self.ui.findChild(QtWidgets.QLabel, "lblTelemRow2")
        # camera
        self._cam_timer: Optional[QtCore.QTimer] = None
        self._cam = None
        self._cam_pixmap_item = None
        self._cam_fit_once = False
        self._cam_fail_cnt = 0
        self._cam_last_open_profile = None  # ("markers"/"road", w,h,fps,dev)
        self._cam_scene_size = None
        self._scene_noindex_set = False
        # fps
        self._fps_t0 = time.monotonic()
        self._fps_n = 0
        self._fps_val = 0.0

        # --- telemetry timer (independent from camera) ---
        self._telem_timer = QtCore.QTimer(self.ui)
        self._telem_timer.setInterval(200)  # 5 Гц, достаточно
        self._telem_timer.timeout.connect(self._update_telem_labels)
        self._telem_timer.start()
        # EMA for road_bin
        self._rb_ema_small = None

        # --- ArUco worker ---
        self._aruco_proc = None
        self._aruco_conn: Optional[Connection] = None
        self._aruco_last = None
        self._aruco_last_ts = 0.0
        self._aruco_waiting = False
        self._aruco_send_ts = 0.0
        self._aruco_last_send_wall = 0.0

        # --- ROI search mode state ---
        self._aruco_search_mode = "scan"     # стартуем с дальняка: скан по кропам
        self._aruco_last_xy = None           # (cx_full, cy_full)
        self._aruco_last_roi = None          # (x0,y0,w,h) последняя ROI отправка

        # scan state
        self._aruco_scan_x0 = 0
        self._aruco_scan_dir = +1
        self._aruco_miss_cnt = 0
        self._aruco_full_seen_t0 = None   # когда начали стабильно видеть в FULL
        self._aruco_track_lost_t0 = None  # когда начали терять в TRACK
        self._aruco_full_hold_t0 = None
        # phone stream (optional)
        self._phone_srv = None
        self._phone_enabled = False

        self._load_cam_calib_into_state()
        self._start_phone_stream_if_enabled()
        self._start_aruco_worker()

        # --- RoadSeg loader (dynamic by state.road_model_path) ---
        self._RoadSeg = None
        self._seg = None
        self._seg_onnx_path = None
        self._seg_sys_path_added = False
        self._seg_last_model_key = None  # used to reload if road_model_path changed

        self.road_follow = RoadFollowController(state)

        # camera starts in current profile
        self._ensure_camera_for_current_mode()
        self._ensure_camera_timer()

        print("[VIDEO] ready", flush=True)

    # ============================================================
    # Public API
    # ============================================================

    def ensure_started(self):
        self._ensure_camera_for_current_mode()
        self._ensure_camera_timer()

    def shutdown(self):
        try:
            if self._cam_timer and self._cam_timer.isActive():
                self._cam_timer.stop()
        except Exception:
            pass

        try:
            if self._cam is not None:
                self._cam.release()
        except Exception:
            pass

        self._cam = None
        try:
            if hasattr(self.state, "_cam"):
                self.state._cam = None
        except Exception:
            pass

        self.remove_camera_overlay()

        # stop aruco worker
        try:
            if self._aruco_conn is not None:
                try:
                    self._aruco_conn.send(None)
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if self._aruco_proc is not None and self._aruco_proc.is_alive():
                self._aruco_proc.terminate()
                self._aruco_proc.join(timeout=0.5)
        except Exception:
            pass

        self._aruco_proc = None
        self._aruco_conn = None
    def set_mode(self, mode: str):
        return self.set_video_mode(mode)
    def set_video_mode(self, mode: str):
        """
        camera/radar view mode (your existing switch). Keep as before:
          - "camera": show camera pixmap in scene
          - "radar":  remove camera overlay, keep radar graphics
        """
        mode = "camera" if mode == "camera" else "radar"
        self.state.video_mode = mode

        if mode == "camera":
            self._cam_fit_once = False
            self._cam_scene_size = None  # <-- чтобы _render_camera заново setSceneRect сделал
        else:
            self.remove_camera_overlay()

        print(f"[DRIVE] video_mode -> {mode}", flush=True)

    def set_nav_mode(self, nav_mode: str):
        """
        Called by DrivePage when user switched MODE_MARKERS/MODE_ROAD/MODE_TRAJ.
        We re-open camera with proper resolution and (re)load seg if needed.
        """
        self.state.nav_mode = nav_mode
        self._ensure_camera_for_current_mode()
        # seg is needed only for road mode (but we keep lazy-load)
        print(f"[DRIVE] nav_mode -> {nav_mode}", flush=True)

    # ============================================================
    # Init helpers
    # ============================================================

    def _start_phone_stream_if_enabled(self):
        try:
            from phone_log_server import start_phone_log_server, phone_log
            if bool(getattr(self.state, "phone_stream", True)):
                host = str(getattr(self.state, "phone_host", "0.0.0.0") or "0.0.0.0")
                port = int(getattr(self.state, "phone_port", 8088) or 8088)
                self._phone_srv = start_phone_log_server(host=host, port=port)
                self._phone_enabled = True
                phone_log("INFO", "phone stream enabled", host=host, port=port)
                print(f"[PHONE] stream: http://<ROVER_IP>:{port}/", flush=True)
        except Exception as e:
            self._phone_srv = None
            self._phone_enabled = False
            print("[PHONE] stream disabled:", e, flush=True)

    def _start_aruco_worker(self):
        try:
            if self._aruco_proc is not None and self._aruco_proc.is_alive():
                return
            ctx = mp.get_context("spawn")
            parent_conn, child_conn = ctx.Pipe(duplex=True)

            from aruco_worker import run as aruco_run
            p = ctx.Process(target=aruco_run, args=(child_conn,), daemon=True)
            p.start()

            self._aruco_proc = p
            self._aruco_conn = parent_conn
            print("[ARUCO] worker started", flush=True)
        except Exception as e:
            self._aruco_proc = None
            self._aruco_conn = None
            print("[ARUCO] worker start error:", e, flush=True)

    def _load_cam_calib_into_state(self):
        st = self.state
        try:
            here = os.path.dirname(__file__)
            calib_path = os.path.join(here, "metki", "camera_calib.npz")

            if os.path.exists(calib_path):
                d = np.load(calib_path)
                K = d["K"]
                D = d["D"]

                st.cam_K = np.ascontiguousarray(K, dtype=np.float64)
                st.cam_D = np.ascontiguousarray(D, dtype=np.float64)
                st.cam_img_size = tuple(int(x) for x in d.get("img_size", (0, 0)))

                print("[CALIB] loaded:", calib_path, "K", st.cam_K.shape, "D", st.cam_D.shape, flush=True)
            else:
                st.cam_K = None
                st.cam_D = None
                print("[CALIB] not found:", calib_path, flush=True)

            if not hasattr(st, "marker_len_m") or float(getattr(st, "marker_len_m", 0.0) or 0.0) <= 1e-6:
                st.marker_len_m = 0.173  # твои печатные метки

            print("[CALIB] marker_len_m:", float(st.marker_len_m), flush=True)
        except Exception as e:
            st.cam_K = None
            st.cam_D = None
            print("[CALIB] load error:", e, flush=True)

    # ============================================================
    # Camera open profiles (4K markers vs 720p road)
    # ============================================================

    def _current_nav_mode(self) -> str:
        return str(getattr(self.state, "nav_mode", MODE_MARKERS) or MODE_MARKERS)

    def _profile_for_mode(self, nav_mode: str) -> Tuple[int, int, int]:
        """
        Returns (w,h,fps) based on nav mode.
        MARKERS: use st.cam_w/cam_h if you set them to 4K, else default 3840x2160
        ROAD:    fixed 1280x720 (as you requested)
        TRAJ:    follow ROAD profile (safe & light), unless you want otherwise
        """
        st = self.state

        if nav_mode == MODE_MARKERS:
            w = int(getattr(st, "cam_w_markers", 3840) or 3840)
            h = int(getattr(st, "cam_h_markers", 2160) or 2160)
            fps = int(getattr(st, "cam_fps_markers", 30) or 30)
            return (w, h, fps)

        # road/traj
        w = int(getattr(st, "cam_w_road", 1280) or 1280)
        h = int(getattr(st, "cam_h_road", 720) or 720)
        fps = int(getattr(st, "cam_fps_road", 30) or 30)
        return (w, h, fps)

    def _ensure_camera_for_current_mode(self) -> bool:
        nav_mode = self._current_nav_mode()
        w, h, fps = self._profile_for_mode(nav_mode)
        dev = getattr(self.state, "cam_device", None)

        prof = (nav_mode, int(w), int(h), int(fps), str(dev))
        if self._cam_last_open_profile == prof and self._cam is not None and self._cam.isOpened():
            return True

        # force reopen with new profile
        try:
            if self._cam is not None:
                self._cam.release()
        except Exception:
            pass
        self._cam = None
        try:
            self.state._cam = None
        except Exception:
            pass

        ok = self._open_camera(dev=dev, w=w, h=h, fps=fps)
        if ok:
            self._cam_last_open_profile = prof
        return ok

    def _open_camera(self, dev, w: int, h: int, fps: int) -> bool:
        """
        Raspberry Pi-friendly camera open:
        - No hardcoded /dev/v4l/* paths.
        - Auto-detect first working /dev/video0..4 (unless dev index provided).
        - Try 4K via direct OpenCV; if fails, try GStreamer pipeline; then fallback to 1080p.
        """

        def _log_cap_selected(video_index: int):
            try:
                print(f"[CAM] selected device: /dev/video{int(video_index)}", flush=True)
            except Exception:
                pass

        def _cap_real_mode(cap: "cv2.VideoCapture") -> tuple[int, int, float]:
            try:
                rw = int(round(float(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0.0)))
                rh = int(round(float(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0.0)))
                rf = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
                return (rw, rh, rf)
            except Exception:
                return (0, 0, 0.0)

        def _try_open_index(index: int, ww: int, hh: int, ff: int) -> tuple[bool, "cv2.VideoCapture|None", "object|None", str]:
            """Direct OpenCV open by index (universal for PC & RPi)."""
            try:
                cap = cv2.VideoCapture(int(index))
                try:
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(ww))
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(hh))
                    cap.set(cv2.CAP_PROP_FPS, int(ff))
                except Exception:
                    pass
                ok, fr = cap.read()
                if (not cap.isOpened()) or (not ok) or (fr is None):
                    try:
                        cap.release()
                    except Exception:
                        pass
                    return (False, None, None, "opencv-index")
                return (True, cap, fr, "opencv-index")
            except Exception:
                return (False, None, None, "opencv-index")

        def _gst_pipeline(video_index: int, ww: int, hh: int, ff: int) -> str:
            """
            GStreamer fallback. We try MJPEG first (common for USB cams), then raw.
            Note: CAP_GSTREAMER must be enabled in OpenCV build for this to work.
            """
            dev_path = f"/dev/video{int(video_index)}"
            # MJPEG → decode → appsink
            return (
                f"v4l2src device={dev_path} ! "
                f"image/jpeg,width={int(ww)},height={int(hh)},framerate={int(ff)}/1 ! "
                f"jpegdec ! videoconvert ! "
                f"appsink drop=1 sync=false max-buffers=1"
            )

        def _try_open_gst(video_index: int, ww: int, hh: int, ff: int) -> tuple[bool, "cv2.VideoCapture|None", "object|None", str]:
            pipe = _gst_pipeline(video_index, ww, hh, ff)
            try:
                cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
                ok, fr = cap.read()
                if (not cap.isOpened()) or (not ok) or (fr is None):
                    try:
                        cap.release()
                    except Exception:
                        pass
                    return (False, None, None, "gstreamer")
                return (True, cap, fr, "gstreamer")
            except Exception:
                return (False, None, None, "gstreamer")

        def _autodetect_index(prefer: Optional[int]) -> Optional[int]:
            """
            Requirement: probe /dev/video0..4 and pick the first camera that works.
            If prefer is set, try it first.
            """
            cand = []
            if prefer is not None:
                cand.append(int(prefer))
            for i in range(5):
                if i not in cand:
                    cand.append(i)

            # lightweight probe first (faster & more compatible)
            for i in cand:
                ok, cap, fr, _ = _try_open_index(i, 640, 480, 30)
                if ok:
                    try:
                        cap.release()
                    except Exception:
                        pass
                    return int(i)
            return None

        # normalize dev to optional index
        prefer_index: Optional[int] = None
        if dev is None:
            prefer_index = None
        elif isinstance(dev, (int, np.integer)):
            prefer_index = int(dev)
        else:
            # accept "0"/"1" from settings/env; ignore any /dev/v4l/* paths
            try:
                prefer_index = int(str(dev).strip())
            except Exception:
                prefer_index = None

        idx = _autodetect_index(prefer_index)
        if idx is None:
            self.state.camera_available = False
            print("[CAM] no camera found in /dev/video0..4", flush=True)
            return False

        _log_cap_selected(idx)
        # persist chosen index so the rest of app sees stable value for this run
        try:
            self.state.cam_device = int(idx)
        except Exception:
            pass

        try:
            # 1) Try 4K direct
            ok, cap, fr, how = _try_open_index(idx, 3840, 2160, 30)
            if not ok:
                time.sleep(0.25)
                ok, cap, fr, how = _try_open_index(idx, 3840, 2160, 30)

            # 2) If 4K direct failed -> try 4K via GStreamer
            if not ok:
                ok, cap, fr, how = _try_open_gst(idx, 3840, 2160, 30)

            # 3) If still failed -> fallback to 1080p direct
            if not ok:
                ok, cap, fr, how = _try_open_index(idx, 1920, 1080, 30)
                if not ok:
                    time.sleep(0.15)
                    ok, cap, fr, how = _try_open_index(idx, 1920, 1080, 30)

            # 4) If still failed -> 1080p via GStreamer
            if not ok:
                ok, cap, fr, how = _try_open_gst(idx, 1920, 1080, 30)

            if not ok or cap is None or fr is None:
                self.state.camera_available = False
                print(f"[CAM] open failed: dev=/dev/video{idx} (tried 4K->gst4K->1080p)", flush=True)
                return False

            self._cam = cap
            self.state._cam = cap
            self.state.camera_available = True
            rw, rh, rf = _cap_real_mode(cap)
            print(
                f"[CAM] opened: dev=/dev/video{idx} via={how} "
                f"real=({rw}x{rh}@{rf:.1f}) frame={getattr(fr,'shape',None)}",
                flush=True,
            )
            return True

        except Exception as e:
            self.state.camera_available = False
            print("[CAM] open error:", e, flush=True)
            return False

    def _ensure_camera_timer(self):
        if self._cam_timer is None:
            self._cam_timer = QtCore.QTimer(self.ui)
            self._cam_timer.setInterval(33)  # ~30 Hz tick
            self._cam_timer.timeout.connect(self._camera_tick)
        if not self._cam_timer.isActive():
            self._cam_timer.start()
    # ============================================================
    # RoadSeg dynamic loading by state.road_model_path
    # ============================================================

    def _resolve_roadseg_from_state(self) -> Tuple[Optional[str], Optional[str]]:
        """
        Uses:
          - state.road_model_path: can be directory OR .onnx file.
        Returns:
          (roadseg_dir, onnx_path)
        """
        st = self.state
        p = str(getattr(st, "road_model_path", "") or "").strip()
        if not p:
            return (None, None)

        p = os.path.expanduser(p)

        if os.path.isfile(p) and p.lower().endswith(".onnx"):
            return (os.path.dirname(p), p)

        if os.path.isdir(p):
            # pick an ONNX inside directory
            # preference: roadseg_*.onnx then any *.onnx
            cand = None
            gl1 = sorted(glob.glob(os.path.join(p, "roadseg*.onnx")))
            if gl1:
                cand = gl1[0]
            else:
                gl2 = sorted(glob.glob(os.path.join(p, "*.onnx")))
                if gl2:
                    cand = gl2[0]
            return (p, cand)

        # unknown path
        return (None, None)

    def _ensure_seg_loaded_if_needed(self):
        # пока мы ещё не вошли в Drive через Next — вообще ничего не грузим
        if not bool(getattr(self.state, "in_drive", False)):
            return

        nav_mode = self._current_nav_mode()
        if nav_mode != MODE_ROAD:
            return
        roadseg_dir, onnx_path = self._resolve_roadseg_from_state()
        model_key = (roadseg_dir or "", onnx_path or "")
        if model_key == self._seg_last_model_key and self._seg is not None:
            return

        self._seg_last_model_key = model_key
        self._seg = None
        self._RoadSeg = None
        self._seg_onnx_path = onnx_path

        # 1) проверяем именно файл модели
        if not onnx_path or (not os.path.isfile(onnx_path)):
            print(f"[SEG] road_model_path invalid (onnx not found): {onnx_path!r}; seg disabled", flush=True)
            return

        # 2) импортируем roadseg из ui/roadseg_work (через sys.path)
        try:
            here = os.path.dirname(__file__)          # .../ui
            code_dir = os.path.abspath(os.path.join(here, "roadseg_work"))  # .../ui/roadseg_work

            if os.path.isdir(code_dir) and (code_dir not in sys.path):
                sys.path.insert(0, code_dir)

            from roadseg import RoadSeg
            self._RoadSeg = RoadSeg
        except Exception as e:
            self._RoadSeg = None
            print("[SEG] import roadseg failed:", e, flush=True)
            return

        # 3) создаём сегментатор
        try:
            self._seg = self._RoadSeg(onnx_path=onnx_path, input_size=(512, 512))
            if getattr(self._seg, "ok", False):
                print(f"[SEG] RoadSeg ready: {onnx_path}", flush=True)
            else:
                print(f"[SEG] RoadSeg not ok (stub?) path={onnx_path}", flush=True)
        except Exception as e:
            self._seg = None
            print("[SEG] init error:", e, flush=True)

        if self._RoadSeg is None:
            return

        if not onnx_path or (not os.path.exists(onnx_path)):
            print("[SEG] ONNX not found in road_model_path; seg disabled", flush=True)
            return

    # ============================================================
    # ArUco detect (non-blocking IPC)
    # ============================================================
    def _detect_aruco(self, frame_bgr):
        st = self.state

        if self._aruco_proc is None or not self._aruco_proc.is_alive():
            self._start_aruco_worker()
            return None

        now = time.monotonic()

        # -----------------------------
        # time params
        # -----------------------------
        track_lost_s = float(getattr(st, "aruco_track_lost_s", 3.0) or 3.0)

        # когда считаем, что "уже близко" и можно переключаться в FULL
        # критерий — площадь corners (в FULL координатах)
        full_area_thr = float(getattr(st, "aruco_full_area_thr", 9000.0) or 9000.0)
        full_hold_s   = float(getattr(st, "aruco_full_hold_s", 0.20) or 0.20)

        # -----------------------------
        # recv (non-blocking)
        # -----------------------------
        got_new = False
        res = None
        try:
            if self._aruco_conn.poll(0.0):
                res = self._aruco_conn.recv()
                got_new = True
        except Exception:
            res = None
            got_new = True

        if got_new:
            self._aruco_last = res
            self._aruco_last_ts = now

            # helper: shift ROI coords -> FULL coords
            def _shift_to_full(mi, roi):
                if roi is None:
                    return mi
                x0, y0, _, _ = roi
                mi = dict(mi)
                mi["cx"] = float(mi.get("cx", 0.0)) + float(x0)
                mi["cy"] = float(mi.get("cy", 0.0)) + float(y0)
                cc = mi.get("corners", None)
                if cc and len(cc) == 4:
                    out = []
                    for p in cc:
                        out.append([float(p[0]) + float(x0), float(p[1]) + float(y0)])
                    mi["corners"] = out
                return mi

            # normalize + shift to FULL
            if res is not None:
                if isinstance(res, dict):
                    self._aruco_last = _shift_to_full(res, self._aruco_last_roi)
                elif isinstance(res, (list, tuple)):
                    tmp = []
                    for x in res:
                        if isinstance(x, dict):
                            tmp.append(_shift_to_full(x, self._aruco_last_roi))
                    self._aruco_last = tmp

            # update last_xy + area
            found = (self._aruco_last is not None)
            if found:
                self._aruco_track_lost_t0 = None

                # take first marker
                mi0 = None
                if isinstance(self._aruco_last, dict):
                    mi0 = self._aruco_last
                elif isinstance(self._aruco_last, (list, tuple)) and self._aruco_last:
                    mi0 = self._aruco_last[0]

                if isinstance(mi0, dict):
                    try:
                        self._aruco_last_xy = (float(mi0.get("cx", 0.0)), float(mi0.get("cy", 0.0)))
                    except Exception:
                        pass

                    # compute marker area from corners in FULL coords
                    area = 0.0
                    try:
                        cc = mi0.get("corners", None)
                        if cc and len(cc) == 4:
                            c32 = np.array(cc, dtype=np.float32).reshape(-1, 2)
                            area = float(cv2.contourArea(c32))
                    except Exception:
                        area = 0.0

                    # decide "close enough" -> switch to FULL (but require small hold to avoid flicker)
                    if area >= full_area_thr:
                        if self._aruco_full_hold_t0 is None:
                            self._aruco_full_hold_t0 = now
                        elif (now - float(self._aruco_full_hold_t0)) >= full_hold_s:
                            self._aruco_search_mode = "full"
                            self._aruco_last_roi = None
                    else:
                        self._aruco_full_hold_t0 = None
                        # если нашли в scan -> сразу в track
                        if self._aruco_search_mode == "scan":
                            self._aruco_search_mode = "track"
            else:
                self._aruco_full_hold_t0 = None
                # miss handling for TRACK only
                if self._aruco_search_mode == "track":
                    if self._aruco_track_lost_t0 is None:
                        self._aruco_track_lost_t0 = now
                    elif (now - float(self._aruco_track_lost_t0)) >= track_lost_s:
                        # потеряли 3 секунды -> обратно в scan
                        self._aruco_search_mode = "scan"
                        self._aruco_track_lost_t0 = None
                        self._aruco_last_xy = None
                        self._aruco_last_roi = None

        # -----------------------------
        # send (rate-limited)
        # -----------------------------
        send_period = float(getattr(st, "aruco_send_period_s", 0.06) or 0.06)
        if (now - float(getattr(self, "_aruco_last_send_wall", 0.0))) < send_period:
            return self._aruco_last

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        H, W = gray.shape[:2]

        wanted_ids = [
            int(getattr(st, "aruco_id_straight_right_pole", 10) or 10),
            int(getattr(st, "aruco_id_straight_left_pole", 15) or 15),
            int(getattr(st, "aruco_id_turn_right", 20) or 20),
            int(getattr(st, "aruco_id_turn_left", 30) or 30),
        ]
        marker_len_m = float(getattr(st, "marker_len_m", 0.173) or 0.173)

        # -----------------------------
        # choose image to send
        # -----------------------------
        mode = str(getattr(self, "_aruco_search_mode", "scan") or "scan")

        # intrinsics
        K0 = getattr(st, "cam_K", None)
        D0 = getattr(st, "cam_D", None)

        img_send = gray
        K_send = K0
        D_send = D0
        self._aruco_last_roi = None

        if mode == "full":
            # FULL_BAND: ищем только в "полосе", как в scan
            top_frac = float(getattr(st, "aruco_scan_top_frac", 0.20) or 0.20)
            bot_frac = float(getattr(st, "aruco_scan_bot_frac", 0.30) or 0.30)
            top_frac = float(_clamp(top_frac, 0.0, 0.45))
            bot_frac = float(_clamp(bot_frac, 0.0, 0.45))

            y0_band = int(H * top_frac)
            y1_band = int(H * (1.0 - bot_frac))
            y0_band = int(_clamp(y0_band, 0, H - 2))
            y1_band = int(_clamp(y1_band, y0_band + 1, H))

            band_h = y1_band - y0_band

            img_send = gray[y0_band:y0_band + band_h, 0:W]
            self._aruco_last_roi = (0, y0_band, W, band_h)

            # K shift only by y0
            if K0 is not None:
                K_send = K0.copy()
                K_send[1, 2] -= float(y0_band)
            else:
                K_send = None
            D_send = D0

        elif mode == "track" and self._aruco_last_xy is not None:
            # tracking ROI around last_xy (в FULL координатах)
            cx, cy = self._aruco_last_xy

            crop_w = int(getattr(st, "aruco_track_w", 1920) or 1920)
            crop_h = int(getattr(st, "aruco_track_h", 1080) or 1080)
            crop_w = min(crop_w, W)
            crop_h = min(crop_h, H)

            x0 = int(_clamp(cx - crop_w * 0.5, 0, W - crop_w))
            y0 = int(_clamp(cy - crop_h * 0.5, 0, H - crop_h))

            img_send = gray[y0:y0 + crop_h, x0:x0 + crop_w]
            self._aruco_last_roi = (x0, y0, crop_w, crop_h)

            # shift K principal point into ROI coords
            if K0 is not None:
                K_send = K0.copy()
                K_send[0, 2] -= float(x0)
                K_send[1, 2] -= float(y0)
            else:
                K_send = None
            D_send = D0
        # ============================
        # SCAN MODE: left + right crop
        # ============================
        elif mode == "scan":
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            H, W = gray.shape[:2]

            # --- vertical band ---
            top_frac = float(getattr(st, "aruco_scan_top_frac", 0.20) or 0.20)
            bot_frac = float(getattr(st, "aruco_scan_bot_frac", 0.30) or 0.30)
            top_frac = float(_clamp(top_frac, 0.0, 0.45))
            bot_frac = float(_clamp(bot_frac, 0.0, 0.45))

            y0 = int(H * top_frac)
            y1 = int(H * (1.0 - bot_frac))
            y0 = int(_clamp(y0, 0, H - 2))
            y1 = int(_clamp(y1, y0 + 1, H))
            band_h = y1 - y0

            # --- horizontal split (4K -> 2x 1920) ---
            half_w = W // 2

            rois = [
                (0,      y0, half_w, band_h),     # LEFT
                (half_w, y0, half_w, band_h),     # RIGHT
            ]

            # round-robin: send LEFT then RIGHT then LEFT...
            phase = int(getattr(self, "_aruco_scan_phase", 0) or 0) % len(rois)
            x0, y0r, w, h = rois[phase]

            img_send = gray[y0r:y0r + h, x0:x0 + w]
            if img_send.size == 0:
                return self._aruco_last

            ok, buf = cv2.imencode(".jpg", img_send, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
            if not ok:
                return self._aruco_last

            # intrinsics: shift principal point into ROI coords
            K0 = getattr(st, "cam_K", None)
            D0 = getattr(st, "cam_D", None)
            if K0 is not None:
                K_send = K0.copy()
                K_send[0, 2] -= float(x0)
                K_send[1, 2] -= float(y0r)
            else:
                K_send = None

            try:
                self._aruco_conn.send((
                    buf.tobytes(),
                    wanted_ids,
                    marker_len_m,
                    K_send,
                    D0,
                ))
                self._aruco_last_roi = (x0, y0r, w, h)
                self._aruco_last_send_wall = now
            except Exception:
                pass

            # переключаем половину на следующий тик
            self._aruco_scan_phase = (phase + 1) % len(rois)

            return self._aruco_last
        ok, buf = cv2.imencode(".jpg", img_send, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            return self._aruco_last

        try:
            self._aruco_conn.send((
                buf.tobytes(),
                wanted_ids,
                marker_len_m,
                K_send,
                D_send,
            ))
            self._aruco_last_send_wall = now
        except Exception:
            pass

        return self._aruco_last
    # ============================================================
    # Marker control 
    # ============================================================
    def _pose_by_done_m(self, done_m: float):
        st = self.state
        pts = getattr(st, "route_pts_px", None) or []
        cum = getattr(st, "route_cum_px", None)
        Lm = float(getattr(st, "route_len_m", 0.0) or 0.0)
        Lpx = float(getattr(st, "route_len_px", 0.0) or 0.0)

        if len(pts) < 2 or not cum or len(cum) != len(pts) or Lm <= 1e-9 or Lpx <= 1e-9:
            return None  # cannot compute

        dm = max(0.0, min(float(done_m), Lm))
        s_px = (dm / Lm) * Lpx

        i = 0
        while i < len(cum) - 1 and float(cum[i + 1]) < s_px:
            i += 1
        i = min(i, len(pts) - 2)

        seg_len_px = max(1e-9, float(cum[i + 1] - cum[i]))
        t = (s_px - float(cum[i])) / seg_len_px
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t

        ax, ay = pts[i]
        bx, by = pts[i + 1]
        x = float(ax + t * (bx - ax))
        y = float(ay + t * (by - ay))
        return (x, y, i, bx, by)
    def reset_pole_snap_anchor(self, reason: str = "start"):
        st = self.state
        st._pole_anchor_route_m = None
        st._pole_anchor_robot_px = None
        st._pole_track_active = False
        st._pole_track_id = None
        st._pole_track_start_ts = 0.0
        st._pole_track_lost_ts = None
        st._pole_last_cx = None
        st._pole_track_cx_anchor = None  # <-- FIX
        print(f"[POLE][SNAP] RESET anchor (reason={reason})", flush=True)
    def _marker_snap_on_pole_pass(self):
        """
        POLE SNAP (правильная семантика):

        - Первый проезд столба:
            anchor_route_m := cur_route_m
            (опционально) anchor_robot_px := robot_px
            LOG: anchor init

        - Каждый следующий проезд столба:
            expected := anchor_route_m + pole_spacing_m
            route_m := expected
            anchor_route_m := expected
            LOG: cur -> expected (delta)

        Ограничение (как ты просил ранее) оставляем:
        - если проехали от anchor меньше pole_snap_min_travel_m -> НЕ снепаем, но логируем.
        """
        st = self.state
        now = time.monotonic()

        pole_spacing_m = float(getattr(st, "pole_spacing_m", 25) or 25)
        pole_spacing_m = float(_clamp(pole_spacing_m, 0.5, 50.0))

        min_travel_for_snap_m = float(getattr(st, "pole_snap_min_travel_m", 15.0) or 15.0)
        min_travel_for_snap_m = float(_clamp(min_travel_for_snap_m, 0.0, 200.0))

        def _get_route_m():
            # ВАЖНО: читаем всегда ОДИН и тот же ключ приоритетно
            for key in ("route_done_m", "route_progress_m", "route_s_m", "robot_route_m"):
                if hasattr(st, key):
                    v = getattr(st, key, None)
                    if v is None:
                        continue
                    try:
                        return float(v)
                    except Exception:
                        pass
            return None

        def _set_route_m(v: float) -> bool:
            # ВАЖНО: пишем тоже приоритетно в один ключ (лучше route_done_m)
            wrote = False
            if hasattr(st, "route_done_m"):
                st.route_done_m = float(v)
                wrote = True
            elif hasattr(st, "route_progress_m"):
                st.route_progress_m = float(v)
                wrote = True
            elif hasattr(st, "route_s_m"):
                st.route_s_m = float(v)
                wrote = True
            elif hasattr(st, "robot_route_m"):
                st.robot_route_m = float(v)
                wrote = True

            # если есть возможность — синхронизируем robot_px/heading по done_m
            try:
                if wrote and hasattr(self, "_set_robot_pose_by_done_m"):
                    self._set_robot_pose_by_done_m(float(v))
            except Exception:
                pass
            return wrote

        cur = _get_route_m()
        if cur is None:
            print("[POLE][SNAP] skip: no route_m in state", flush=True)
            return

        # anchor = позиция последнего проезда столба в метрах по маршруту
        if not hasattr(st, "_pole_anchor_route_m"):
            st._pole_anchor_route_m = None
        if not hasattr(st, "_pole_anchor_robot_px"):
            st._pole_anchor_robot_px = None

        if st._pole_anchor_route_m is None:
            st._pole_anchor_route_m = float(cur)
            st._pole_anchor_robot_px = getattr(st, "robot_px", None)
            print(
                f"[POLE][SNAP] anchor init: anchor_route_m={st._pole_anchor_route_m:.2f}m "
                f"robot_px={st._pole_anchor_robot_px}",
                flush=True,
            )
            st._marker_last_pole_pass_ts = float(now)
            return

        anchor = float(st._pole_anchor_route_m)
        traveled = float(cur - anchor)

        # ограничение на минимум проезда между "столбами"
        if traveled < min_travel_for_snap_m:
            print(
                f"[POLE][SNAP] NO-SNAP (travel<{min_travel_for_snap_m:.1f}m): "
                f"cur={cur:.2f}m anchor={anchor:.2f}m traveled={traveled:.2f}m -> REANCHOR",
                flush=True,
            )

            # ✅ ВАЖНО: считаем это "последним проездом столба", но без SNAP
            # чтобы следующий столб не считался от старого anchor (иначе будет скачок "на 2 столба назад")
            st._pole_anchor_route_m = float(cur)
            st._pole_anchor_robot_px = getattr(st, "robot_px", None)

            st._marker_last_pole_pass_ts = float(now)
            return

        expected = float(anchor + pole_spacing_m)
        delta = float(expected - cur)

        ok = _set_route_m(expected)

        print(
            f"[POLE][SNAP] SNAP: cur={cur:.2f}m -> expected={expected:.2f}m "
            f"(delta={delta:+.2f}m) spacing={pole_spacing_m:.2f}m ok={ok}",
            flush=True,
        )

        # Сдвигаем якорь строго на spacing
        st._pole_anchor_route_m = float(expected)
        st._pole_anchor_robot_px = getattr(st, "robot_px", None)
        st._marker_last_pole_pass_ts = float(now)
    def _set_robot_pose_by_done_m(self, done_m: float) -> bool:
        """
        Жестко выставляет robot_px / route_progress_idx / heading по done_m
        на основе route_pts_px + route_cum_px (как в start_route_animation).
        """
        st = self.state

        pts = getattr(st, "route_pts_px", None) or []
        if len(pts) < 2:
            return False

        Lm = float(getattr(st, "route_len_m", 0.0) or 0.0)
        Lpx = float(getattr(st, "route_len_px", 0.0) or 0.0)
        cum = getattr(st, "route_cum_px", None)

        if Lm <= 1e-9 or Lpx <= 1e-9 or (not cum) or len(cum) != len(pts):
            return False

        # clamp done_m
        dm = float(done_m)
        if dm < 0.0:
            dm = 0.0
        if dm > Lm:
            dm = Lm

        # done_m -> s_px along polyline
        s_px = (dm / Lm) * Lpx

        # find segment index i such that cum[i] <= s_px <= cum[i+1]
        i = 0
        while i < len(cum) - 1 and cum[i + 1] < s_px:
            i += 1
        i = min(i, len(pts) - 2)

        seg_len_px = max(1e-9, float(cum[i + 1] - cum[i]))
        t = (s_px - float(cum[i])) / seg_len_px
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t

        ax, ay = pts[i]
        bx, by = pts[i + 1]

        x = float(ax + t * (bx - ax))
        y = float(ay + t * (by - ay))

        st.route_progress_idx = int(i)
        st.route_done_m = float(dm)
        st.robot_px = (x, y)
        st.robot_heading_rad = math.atan2(float(by - ay), float(bx - ax))
        return True
    def _marker_drive(self, frame_bgr) -> bool:
        """
        Marker-only controller.

        Требования (актуальные):
        - Нет min времени поворота.
        - Нет slew/нарастания: поворот всегда с постоянной скоростью (фиксированный steer).
        - Выход из поворота: 3 подряд кадра со "straight" (id 10 или 15).
        - Нет лимита "если метку не видим N секунд" — НИЧЕГО не отдаём дороге по времени.
        - Если метку не видим: едем прямо (base/base).
        - При движении прямо: просто motors_set(base, base, None).
        - При повороте: крутим до тех пор, пока не встретили знак "straight".
        - POLE SNAP: когда трекаемый straight-столб пропал из кадра (miss_cnt>=N),
        тогда один раз делаем SNAP.
        N = 10 кадров.
        Ищем/принимаем "тот же столб" только в зоне, где он был последний раз
        (по cx в окне ±gate_frac*W).
        Возвращает True всегда (мы всегда управляем моторами в marker-mode).
        """
        st = self.state
        now = time.monotonic()
        H, W = frame_bgr.shape[:2]

        # -----------------------------
        # PWM params
        # -----------------------------
        base = int(getattr(st, "pwm_base_us", 1750) or 1750)
        base = int(_clamp(base, 1500, 2000))

        max_delta = int(getattr(st, "max_delta_pwm", 120) or 120)
        max_delta = int(_clamp(max_delta, 10, 800))

        # -----------------------------
        # ids
        # -----------------------------
        straight_right_id = int(getattr(st, "aruco_id_straight_right_pole", 10) or 10)
        straight_left_id  = int(getattr(st, "aruco_id_straight_left_pole", 15) or 15)
        turn_right_id     = int(getattr(st, "aruco_id_turn_right", 20) or 20)
        turn_left_id      = int(getattr(st, "aruco_id_turn_left", 30) or 30)

        # -----------------------------
        # init state (one-time)
        # -----------------------------
        if not hasattr(st, "_pole_track_lost_ts"):
            st._pole_track_lost_ts = None
        if not hasattr(st, "_pole_track_last_seen_ts"):
            st._pole_track_last_seen_ts = 0.0
        if not hasattr(st, "_pole_track_acquired"):
            st._pole_track_acquired = False
        if not hasattr(st, "marker_turn_active"):
            st.marker_turn_active = False
        if not hasattr(st, "marker_turn_dir"):
            st.marker_turn_dir = None
        if not hasattr(st, "_marker_turn_exit_cnt"):
            st._marker_turn_exit_cnt = 0
        if not hasattr(st, "_turn_lock_id"):
            st._turn_lock_id = None
        if not hasattr(st, "_turn_lock_dir"):
            st._turn_lock_dir = None
        if not hasattr(st, "_turn_trig_cnt"):
            st._turn_trig_cnt = 0
        if not hasattr(st, "marker_last_steer"):
            st.marker_last_steer = 0.0
        if not hasattr(st, "marker_active"):
            st.marker_active = False
        if not hasattr(st, "turn_execute"):
            st.turn_execute = False

        # --- snap-to-poles (graph correction) ---
        if not hasattr(st, "_marker_snap_inited"):
            st._marker_snap_inited = False
        if not hasattr(st, "_marker_snap_anchor_done_m"):
            st._marker_snap_anchor_done_m = None
        if not hasattr(st, "_marker_snap_after_turn"):
            st._marker_snap_after_turn = False
        if not hasattr(st, "_marker_last_pole_pass_ts"):
            st._marker_last_pole_pass_ts = 0.0

        # --- pole tracking (snap only when marker leaves screen) ---
        if not hasattr(st, "_pole_track_active"):
            st._pole_track_active = False
        if not hasattr(st, "_pole_track_id"):
            st._pole_track_id = None
        if not hasattr(st, "_pole_track_start_ts"):
            st._pole_track_start_ts = 0.0
        # last seen zone anchor (legacy) + frozen tracker anchor (fix)
        if not hasattr(st, "_pole_last_cx"):
            st._pole_last_cx = None
        if not hasattr(st, "_pole_track_cx_anchor"):
            st._pole_track_cx_anchor = None  # <-- FIX: frozen cx of current physical pole

        # -----------------------------
        # current marker (from state; already filled by _apply_marker_to_state)
        # -----------------------------
        seen = bool(getattr(st, "marker_seen", False))
        mid = int(getattr(st, "marker_id", -1) or -1)
        dist_m = getattr(st, "marker_dist_m", None)
        tv = getattr(st, "marker_tvec", None)
        cx = float(getattr(st, "marker_cx", W * 0.5) or (W * 0.5))

        # -----------------------------
        # helpers
        # -----------------------------
        def _dist_is_ok(d):
            if d is None:
                return (False, 1e9)
            try:
                v = float(d)
            except Exception:
                return (False, 1e9)
            if not (0.05 < v < 50.0):
                return (False, 1e9)
            return (True, v)

        def _apply_steer_raw(steer01: float):
            # clamp raw
            steer01 = float(np.clip(steer01, -1.0, +1.0))

            # --- dt for smoothing ---
            last_ts = float(getattr(st, "_marker_steer_last_ts", 0.0) or 0.0)
            dt = 0.0 if last_ts <= 0.0 else float(now - last_ts)
            if dt <= 1e-4 or dt > 0.5:
                dt = 1.0 / 30.0  # fallback ~30Hz
            st._marker_steer_last_ts = float(now)

            # --- 1) low-pass filter (EMA) ---
            # marker_steer_tau_s: 0.15..0.45 обычно норм, дальняк -> больше
            tau = float(getattr(st, "marker_steer_tau_s", 0.28) or 0.28)
            tau = float(_clamp(tau, 0.05, 1.00))
            alpha = float(dt / (tau + dt))

            prev = float(getattr(st, "_marker_steer_f", 0.0) or 0.0)
            filt = prev + alpha * (steer01 - prev)

            # --- 2) step limit (slew) on filtered steer ---
            # marker_steer_step: максимум изменения steer за тик (0.03..0.12)
            step = float(getattr(st, "marker_steer_step", 0.06) or 0.06)
            step = float(_clamp(step, 0.01, 0.25))
            df = float(filt - prev)
            if df > step:
                filt = prev + step
            elif df < -step:
                filt = prev - step

            # store
            st._marker_steer_f = float(filt)

            # to PWM
            steer01 = float(np.clip(filt, -1.0, +1.0))
            delta = int(np.clip(steer01 * max_delta, -max_delta, max_delta))
            motors_set(st, base + delta, base - delta, None)

            # HUD/debug
            st.marker_last_steer = steer01

        def _go_straight():
            motors_set(st, base, base, None)
            st.marker_last_steer = 0.0
            st.marker_active = True
            st.turn_execute = False

        def _is_straight_id(m):
            return int(m) in (straight_left_id, straight_right_id)

        def _is_turn_id(m):
            return int(m) in (turn_left_id, turn_right_id)

        def _in_last_zone(cx_now: float) -> bool:
            """
            FIX: "тот же столб" ищем только вокруг FROZEN якоря текущего трекинга.
            Пока _pole_track_active=True, cx_anchor НЕ переносится на новый столб.
            """
            gate_frac = float(getattr(st, "pole_same_gate_frac", 0.15) or 0.15)
            gate_frac = float(_clamp(gate_frac, 0.05, 0.40))
            gate_px = gate_frac * float(W)

            # если трекинг активен — используем frozen anchor
            cx0 = getattr(st, "_pole_track_cx_anchor", None) if bool(getattr(st, "_pole_track_active", False)) else getattr(st, "_pole_last_cx", None)

            if cx0 is None:
                return True
            try:
                return abs(float(cx_now) - float(cx0)) <= gate_px
            except Exception:
                return True

        # =========================================================
        # 1) TURN MODE: rotate until we see STRAIGHT 3 frames подряд
        # =========================================================
        if bool(getattr(st, "marker_turn_active", False)):
            exit_need = 3

            if seen and _is_straight_id(mid):
                st._marker_turn_exit_cnt = int(getattr(st, "_marker_turn_exit_cnt", 0) or 0) + 1
            else:
                st._marker_turn_exit_cnt = 0

            if st._marker_turn_exit_cnt >= exit_need:
                st.marker_turn_active = False
                st.marker_turn_dir = None
                st._marker_turn_exit_cnt = 0
                st._turn_lock_id = None
                st._turn_lock_dir = None
                st._turn_trig_cnt = 0
                st.turn_execute = False

                # ВАЖНО: после РЕАЛЬНОГО завершения поворота разрешаем отпустить граф
                if bool(getattr(st, "graph_freeze_enabled", False)):
                    st.graph_freeze_release = True

                _go_straight()
                return True

            tdir = str(getattr(st, "marker_turn_dir", "") or "")
            sign = -1.0 if tdir == "left" else (+1.0 if tdir == "right" else 0.0)

            turn_steer = float(getattr(st, "marker_turn_steer", 1.0) or 1.0)
            turn_steer = float(_clamp(turn_steer, 0.05, 1.0))

            _apply_steer_raw(sign * turn_steer)
            st.marker_active = True
            st.turn_execute = True
            return True

        # =========================================================
        # 2) POLE TRACK: stable acquire + time-based loss (same ID + same zone)
        # =========================================================
        loss_s = float(getattr(st, "pole_loss_s", 3.0) or 3.0)
        loss_s = float(_clamp(loss_s, 0.2, 10.0))

        # сколько кадров подряд нужно увидеть столб, чтобы считать "захватили"
        acq_need = int(getattr(st, "pole_acquire_need_frames", 3) or 3)
        acq_need = int(_clamp(acq_need, 1, 10))

        # небольшая защита от морганий детектора
        grace_s = float(getattr(st, "pole_loss_grace_s", 0.25) or 0.25)
        grace_s = float(_clamp(grace_s, 0.0, 2.0))

        # init доп. полей (один раз)
        if not hasattr(st, "_pole_track_seen_cnt"):
            st._pole_track_seen_cnt = 0
        if not hasattr(st, "_pole_track_last_seen_ts"):
            st._pole_track_last_seen_ts = 0.0
        if not hasattr(st, "_pole_track_acquired"):
            st._pole_track_acquired = False

        if bool(getattr(st, "_pole_track_active", False)):
            tid = getattr(st, "_pole_track_id", None)

            # ВАЖНО: "тот же столб" = тот же ID + в своей зоне
            tracker_seen = bool(
                (tid is not None)
                and seen
                and (int(mid) == int(tid))
                and _in_last_zone(cx)
            )

            if tracker_seen:
                # копим подряд виденные кадры
                st._pole_track_seen_cnt = int(getattr(st, "_pole_track_seen_cnt", 0) or 0) + 1
                st._pole_track_last_seen_ts = float(now)

                # ВАЖНО: пока НЕ acquired — можем подправлять anchor (чтобы захват был стабильный)
                if not bool(getattr(st, "_pole_track_acquired", False)):
                    st._pole_track_cx_anchor = float(cx)
                    st._pole_last_cx = float(cx)  # legacy для дебага/визуала

                # как только "захватили" — ЗАМОРАЖИВАЕМ anchor (дальше не переносим никуда)
                if (not bool(getattr(st, "_pole_track_acquired", False))) and (st._pole_track_seen_cnt >= acq_need):
                    st._pole_track_acquired = True
                    st._pole_track_lost_ts = None
                    # anchor уже выставлен выше; дальше НЕ обновляем
                    print(f"[POLE] ACQUIRED id={tid} cx_anchor={float(st._pole_track_cx_anchor):.1f}", flush=True)

                # пока видим — потери нет
                st._pole_track_lost_ts = None

            else:
                # если ещё не "acquired" — НЕ начинаем потерю вообще (это убирает моментальный LOST start)
                if not bool(getattr(st, "_pole_track_acquired", False)):
                    # сброс подряд виденных кадров, но трек активен (ждём нормального захвата)
                    st._pole_track_seen_cnt = 0

                else:
                    # acquired -> можно начинать потерю, но с grace
                    last_seen = float(getattr(st, "_pole_track_last_seen_ts", 0.0) or 0.0)
                    if last_seen > 0.0 and (now - last_seen) < grace_s:
                        pass
                    else:
                        # стартуем потерю ОДИН раз
                        if getattr(st, "_pole_track_lost_ts", None) is None:
                            st._pole_track_lost_ts = float(now)

                        lost_dt = float(now - float(st._pole_track_lost_ts))
                        if lost_dt >= loss_s:
                            print(f"[POLE] lost {lost_dt:.2f}s >= {loss_s:.2f}s -> SNAP", flush=True)
                            # LOSS CONFIRMED -> SNAP ONCE
                            print(f"[POLE] lost {lost_dt:.2f}s >= {loss_s:.2f}s -> SNAP", flush=True)

                            st._pole_track_active = False
                            st._pole_track_id = None
                            st._pole_track_start_ts = 0.0
                            st._pole_last_cx = None
                            st._pole_track_cx_anchor = None  
                            st._pole_track_lost_ts = None
                            st._pole_track_seen_cnt = 0
                            st._pole_track_last_seen_ts = 0.0
                            st._pole_track_acquired = False

                            st._marker_last_pole_pass_ts = float(now)
                            self._marker_snap_on_pole_pass()
        # =========================================================
        # 3) If marker not seen: ALWAYS straight (no timeouts, no fallback)
        #    (pole snap handled выше, отдельно)
        # =========================================================
        if not seen:
            _go_straight()
            return True

        # =========================================================
        # 4) TURN marker (20/30): approach until dist<=trig then enter turn mode
        # =========================================================
        if _is_turn_id(mid):
            # при поворотных метках — обнуляем "запоминание" столбов, как ты просил
            st._pole_track_active = False
            st._pole_track_id = None
            st._pole_track_start_ts = 0.0
            st._pole_track_lost_ts = None
            st._pole_last_cx = None
            if st._turn_lock_id is None:
                st._turn_lock_id = int(mid)
                st._turn_lock_dir = ("left" if int(mid) == turn_left_id else "right")
                st._turn_trig_cnt = 0

            # --- NEW: if freeze active -> enforce graph direction ---
            if self._freeze_is_active() and self._freeze_is_allowed_now():
                gdir = self._graph_turn_dir_now()          # left/right (real turn)
                adir = self._aruco_turn_dir_from_id(mid)   # left/right from 20/30
                if gdir in ("left", "right") and adir in ("left", "right") and (gdir != adir):
                    # метка "не про этот поворот" -> игнорим 20/30
                    _go_straight()
                    return True

            if st._turn_lock_id is not None and int(mid) != int(st._turn_lock_id):
                _go_straight()
                return True

            trig_m = float(getattr(st, "marker_turn_trigger_m", 5.0) or 5.0)
            trig_m = float(_clamp(trig_m, 0.2, 30.0))

            okd, dv = _dist_is_ok(dist_m)
            if okd and (dv <= trig_m):
                st._turn_trig_cnt = int(getattr(st, "_turn_trig_cnt", 0) or 0) + 1
            else:
                st._turn_trig_cnt = 0

            if st._turn_trig_cnt >= 2:
                st.marker_turn_active = True
                st.marker_turn_dir = st._turn_lock_dir
                st._marker_turn_exit_cnt = 0
                st.turn_execute = True

                st._marker_snap_after_turn = True
                st._marker_snap_inited = False
                st._marker_snap_anchor_done_m = None
                if bool(getattr(st, "graph_freeze_enabled", False)):
                    st.graph_freeze_engaged = True
                tdir = str(st.marker_turn_dir or "")
                sign = -1.0 if tdir == "left" else (+1.0 if tdir == "right" else 0.0)
                turn_steer = float(getattr(st, "marker_turn_steer", 1.0) or 1.0)
                turn_steer = float(_clamp(turn_steer, 0.05, 1.0))
                _apply_steer_raw(sign * turn_steer)
                st.marker_active = True
                return True

            max_app = float(getattr(st, "marker_approach_max_steer", 0.45) or 0.45)
            max_app = float(_clamp(max_app, 0.05, 1.0))

            if tv is not None:
                try:
                    x = float(tv[0])
                except Exception:
                    x = 0.0
                kp_app = float(getattr(st, "marker_approach_kp", 0.60) or 0.60)
                steer_cmd = float(np.clip(kp_app * x, -max_app, +max_app))
                _apply_steer_raw(steer_cmd)
                st.marker_active = True
                st.turn_execute = False
                return True

            dead_px = float(getattr(st, "marker_approach_dead_px", 14.0) or 14.0)
            kp_px = float(getattr(st, "marker_approach_kp_px", 1.6) or 1.6)

            err_px = float(cx - 0.5 * W)
            if abs(err_px) < dead_px:
                err_px = 0.0
            steer_cmd = float(np.clip(kp_px * (err_px / max(1.0, W)), -max_app, +max_app))
            _apply_steer_raw(steer_cmd)
            st.marker_active = True
            st.turn_execute = False
            return True

        # =========================================================
        # 5) STRAIGHT markers (10/15): держим боковое расстояние (по метрам)
        #    + START POLE TRACK (только один раз) и только в зоне
        # =========================================================
        if _is_straight_id(mid):
            side = "right" if int(mid) == straight_right_id else "left"
            st.marker_follow_side = side

            if tv is None:
                _go_straight()
                return True

            try:
                x = float(tv[0])
            except Exception:
                _go_straight()
                return True

            target_side = float(getattr(st, "marker_target_side_m", 2.0) or 2.0)
            target_side = float(_clamp(target_side, 0.2, 10.0))

            dead_m    = float(getattr(st, "marker_deadband_m", 0.15) or 0.15)
            kp        = float(getattr(st, "marker_kp", 0.2) or 0.2)
            max_steer = float(getattr(st, "marker_max_steer", 0.35) or 0.35)

            x_des = (+target_side) if side == "right" else (-target_side)
            err = float(x - x_des)
            if abs(err) < dead_m:
                err = 0.0

            steer_cmd = float(np.clip(kp * err, -max_steer, +max_steer))
            _apply_steer_raw(steer_cmd)
            st.marker_active = True
            st.turn_execute = False

            # --- pole tracking start: только если трекинг не активен ---
            if not bool(getattr(st, "_pole_track_active", False)):
                st._pole_track_active = True
                st._pole_track_id = int(mid)
                st._pole_track_start_ts = float(now)

                # FIX: frozen anchor for this physical pole
                st._pole_track_cx_anchor = float(cx)
                st._pole_last_cx = float(cx)  # legacy/debug

                # ЖЁСТКО обнуляем всю внутреннюю механику трекера
                st._pole_track_lost_ts = None
                st._pole_track_seen_cnt = 0
                st._pole_track_last_seen_ts = 0.0
                st._pole_track_acquired = False

                print(f"[POLE] track start id={mid} cx_anchor={float(st._pole_track_cx_anchor):.1f}", flush=True)
            else:
                # FIX: ничего не переносим на "новый" столб.
                # При active tracking мы НЕ обновляем cx_anchor здесь вообще.
                # Любое обновление разрешено только в tracker_seen И ТОЛЬКО до acquired.
                pass

            return True

        # =========================================================
        # 6) Любой другой marker id: просто прямо
        # =========================================================
        _go_straight()
        return True
    # ============================================================
    # UI indicators
    # ============================================================

    def _update_road_indicator(self):
        try:
            from status import set_indicator
        except Exception:
            return

        # try common names
        ind = self.ui.findChild(QtWidgets.QLabel, "indRoad")
        if ind is None:
            # fallback if UI name differs (but NOT battery)
            ind = self.ui.findChild(QtWidgets.QLabel, "indRoadSeg")
        if ind is None:
            return

        det = bool(getattr(self.state, "road_detected", False))
        set_indicator(ind, "ok" if det else "bad")
    def _fps_update(self) -> float:
        now = time.monotonic()
        self._fps_n += 1

        dt = now - float(self._fps_t0 or now)
        if dt >= 0.5:
            self._fps_val = float(self._fps_n) / float(dt)
            self._fps_n = 0
            self._fps_t0 = now
        return float(self._fps_val or 0.0)
    def _update_telem_labels(self):
        if not self._lblTelemRow1 or not self._lblTelemRow2:
            return

        st = self.state

        # ===== FPS =====
        if bool(getattr(st, "camera_available", False)):
            fps = float(getattr(self, "_fps_val", 0.0) or 0.0)
            fps_txt = f"{fps:4.1f}"
        else:
            fps_txt = "none"

        # ===== SPEED =====
        v = getattr(st, "speed_mps", None)
        try:
            v = float(v) if v is not None else None
        except Exception:
            v = None
        v_txt = "—" if v is None else f"{v:.2f}"

        # ===== STEER =====
        nav = str(getattr(st, "nav_mode", "") or "")
        if nav == "road":
            steer = float(getattr(st, "_rf_steer", 0.0) or 0.0)
        else:
            steer = float(getattr(st, "marker_last_steer", 0.0) or 0.0)

        # ===== PWM =====
        l_pwm = getattr(st, "l_pwm", None)
        r_pwm = getattr(st, "r_pwm", None)

        if l_pwm is not None and r_pwm is not None:
            self._lblTelemRow1.setText(
                f"FPS {fps_txt}   V {v_txt} m/s   PWM {int(l_pwm)}/{int(r_pwm)}"
            )
        else:
            self._lblTelemRow1.setText(
                f"FPS {fps_txt}   V {v_txt} m/s"
            )

        self._lblTelemRow2.setText(f"STEER {steer:+.2f}")
    def _update_turn_lr_ui(self):
        st = self.state
        indL = self.ui.findChild(QtWidgets.QLabel, "indTurnL")
        indR = self.ui.findChild(QtWidgets.QLabel, "indTurnR")
        if not indL or not indR:
            return

        now = time.monotonic()
        cam_ts = float(getattr(st, "_cam_junc_last_seen_ts", 0.0) or 0.0)
        cam_recent = (cam_ts > 0.0) and ((now - cam_ts) <= float(getattr(st, "cam_recent_s", 15) or 15))

        lr = getattr(st, "_cam_junc_seen_lr", None)

        has_left = has_right = False
        if cam_recent and lr and len(lr) >= 2:
            has_left = bool(lr[0])
            has_right = bool(lr[1])

        indL.setStyleSheet(
            "background:#2d6; border-radius:9px; border:1px solid #777;" if has_left
            else "background:#bdbdbd; border-radius:9px; border:1px solid #777;"
        )
        indR.setStyleSheet(
            "background:#2d6; border-radius:9px; border:1px solid #777;" if has_right
            else "background:#bdbdbd; border-radius:9px; border:1px solid #777;"
        )

    # ============================================================
    # Tick
    # ============================================================
    def _aruco_turn_dir_from_id(self, mid_int: int) -> Optional[str]:
        st = self.state
        turn_right_id = int(getattr(st, "aruco_id_turn_right", 20) or 20)
        turn_left_id  = int(getattr(st, "aruco_id_turn_left", 30) or 30)

        mid_int = int(mid_int)
        if mid_int == turn_left_id:
            return "left"
        if mid_int == turn_right_id:
            return "right"
        return None

    def _graph_turn_dir_now(self) -> Optional[str]:
        st = self.state
        v = getattr(st, "next_turn_dir", None)
        v = str(v) if v is not None else None
        return v if v in ("left", "right", "straight") else None

    def _freeze_is_active(self) -> bool:
        st = self.state
        return bool(getattr(st, "graph_freeze_enabled", False))

    def _freeze_is_allowed_now(self) -> bool:
        """
        Marker-mode: никакого roadbin/cam_junc_seen.
        Freeze релевантен, если routing выбрал активный junction и это реальный поворот.
        """
        st = self.state
        if getattr(st, "_active_turn_j_id", None) is None:
            return False
        return self._graph_turn_dir_now() in ("left", "right")
    def _camera_tick(self):
        # keep camera profile consistent with nav mode
        if not self._ensure_camera_for_current_mode():
            return

        if self._cam is None or (not self._cam.isOpened()):
            return

        ok, frame = self._cam.read()
        if not ok or frame is None:
            self._cam_fail_cnt += 1
            if self._cam_fail_cnt >= 10:
                print("[CAM] read failed x10 -> reopen", flush=True)
                try:
                    self._cam.release()
                except Exception:
                    pass
                self._cam = None
                try:
                    self.state._cam = None
                except Exception:
                    pass
                self.state.camera_available = False
                self._cam_fail_cnt = 0
            return

        self._cam_fail_cnt = 0
        self._fps_update()
        st = self.state
        nav_mode = self._current_nav_mode()

        # dataset raw
        if self.dataset is not None:
            try:
                self.dataset.maybe_capture(frame_bgr=frame, source="cam_raw")
            except Exception:
                pass

        # ============================================================
        # MODE_TRAJ: just render camera (trajectory controller handles motors)
        # ============================================================
        if nav_mode == MODE_TRAJ:
            # marker detect can still be useful visually if you want:
            if bool(getattr(st, "marker_mode", True)):
                m = self._detect_aruco(frame)
                self._apply_marker_to_state(m, frame)
            return self._render_camera(frame, mask=None, road_bin=None, road_support=0.0, thr=0.0)

        # ============================================================
        # MODE_MARKERS: ArUco + optional marker drive
        # ============================================================
        if nav_mode == MODE_MARKERS:
            # ВАЖНО: в marker-mode всегда обновляем подсказку поворота по графу
            # (она же подготавливает _junc_proj и может включать freeze-режим)
            try:
                update_junction_turn_hint(st)
            except Exception as e:
                print("[MARKERS] update_junction_turn_hint error:", e, flush=True)

            m = None
            if bool(getattr(st, "marker_mode", True)):
                m = self._detect_aruco(frame)
            self._apply_marker_to_state(m, frame)

            did_marker = False

            is_run = bool(getattr(st, "is_running", False))
            is_manual = bool(getattr(st, "manual_drive", False))
            is_traj = bool(getattr(st, "trajectory_mode", False))

            # 1 Hz debug: почему marker_drive не зовём / что видим
            if not hasattr(st, "_dbg_markers_last_ts"):
                st._dbg_markers_last_ts = 0.0
            now = time.monotonic()
            if (now - float(st._dbg_markers_last_ts or 0.0)) >= 1.0:
                st._dbg_markers_last_ts = now

            if is_run and (not is_manual) and (not is_traj):
                did_marker = self._marker_drive(frame)

            return self._render_camera(frame, mask=None, road_bin=None, road_support=0.0, thr=0.0)

        # ============================================================
        # MODE_ROAD: Segmentation + road_follow (+ optional rails)
        # ============================================================
        if nav_mode == MODE_ROAD:
            self._ensure_seg_loaded_if_needed()

            use_seg = (self._seg is not None) and bool(getattr(self._seg, "ok", False))
            if not use_seg:
                st.road_detected = False
                st.road_state = "no_road"
                st.road_on = False
                self._update_road_indicator()
                return self._render_camera(frame, mask=None, road_bin=None, road_support=0.0, thr=0.0)

            try:
                mask = self._seg.infer(frame)
            except Exception as e:
                print("[CAM] seg infer error:", e, flush=True)
                mask = None

            if mask is None or getattr(mask, "size", 0) == 0:
                st.road_detected = False
                st.road_state = "no_road"
                st.road_on = False
                self._update_road_indicator()
                return self._render_camera(frame, mask=None, road_bin=None, road_support=0.0, thr=0.0)

            # routing hint
            try:
                update_junction_turn_hint(st)
            except Exception as e:
                print("[CAM] update_junction_turn_hint error:", e, flush=True)

            self._update_turn_lr_ui()

            road_bin, thr, crop_y0_full = self._build_road_bin_from_mask(mask)

            # lr (turn branches)
            try:
                lr_raw = detect_turn_branches_from_roadbin(st, road_bin)
                lr = update_cam_lr_debounced(st, lr_raw)
                st._cam_junc_seen_lr = lr
                if (lr[0] or lr[1] or lr[2]):
                    st._cam_junc_last_seen_ts = time.monotonic()
            except Exception:
                lr = getattr(st, "_cam_junc_seen_lr", None)

            # dataset seg capture
            if self.dataset is not None:
                try:
                    self.dataset.maybe_capture(seg_mask=mask, road_bin=road_bin, source="seg")
                except Exception:
                    pass

            rails_enabled = bool(getattr(st, "rails_enabled", False))

            if rails_enabled:
                compute_main_road_from_roadbin(st, road_bin, int(crop_y0_full))
                update_rail_anchors_from_roadbin(st, road_bin, int(crop_y0_full))
            else:
                st._main_line = None
                st._rail_mask_L_full = None
                st._rail_mask_R_full = None
                st._rail_mask_L_crop = None
                st._rail_mask_R_crop = None

            # road_follow always updates (it can ignore rails)
            try:
                self.road_follow.update(road_bin)
            except Exception as e:
                print("[CAM] road_follow error:", e, flush=True)

            # snap (use road_bin_crop)
            try:
                maybe_snap_robot_to_junction_by_camera(
                    st,
                    mask,
                    search_ahead_m=float(getattr(st, "search_ahead_m", 22.0) or 22.0),
                    search_behind_m=float(getattr(st, "search_behind_m", 18.0) or 18.0),
                    snap_before_m=float(getattr(st, "snap_before_m", 8.0) or 8.0),
                    thr=float(getattr(st, "seg_thr", 0.60) or 0.60),
                    road_bin_crop=road_bin,
                )
            except Exception as e:
                print("[CAM] maybe_snap_robot_to_junction_by_camera error:", e, flush=True)

            # road_detected
            support = self._compute_road_support(st, road_bin)
            st.road_detected = bool(support >= float(getattr(st, "road_detect_min_support", 0.05) or 0.05))
            st.road_support = float(support)
            st.road_on = bool(st.road_detected)
            st.road_state = "on_road" if st.road_detected else "no_road"
            self._update_road_indicator()

            # IMPORTANT: motors control only if running and NOT manual
            if bool(getattr(st, "is_running", False)) and (not bool(getattr(st, "manual_drive", False))):
                # road_follow controller should set st._rf_steer etc; if you already apply motors elsewhere keep it there.
                # If you want here: apply steer -> motors_set
                # (I leave it to your RoadFollowController; no new side-effects here.)
                pass

            return self._render_camera(frame, mask=mask, road_bin=road_bin, road_support=support, thr=thr)

        # fallback
        return self._render_camera(frame, mask=None, road_bin=None, road_support=0.0, thr=0.0)

    # ============================================================
    # Marker state write (freshness + bbox cleanup)
    # ============================================================

    def _apply_marker_to_state(self, m, frame_bgr):
        """
        Пишем в state "один выбранный маркер" (closest / priority).
        Требования:
        - Принимаем список маркеров или один маркер.
        - Если маркеров несколько — выбираем ближайший (min dist_m),
        но:
            * если активен pole-tracking -> предпочитаем straight (10/15) в зоне last_cx
            * turn (20/30) имеет приоритет над straight, если он уже "залочен" или близко
        - Если маркера нет -> аккуратно чистим.
        """
        st = self.state
        now = time.monotonic()
        H, W = frame_bgr.shape[:2]

        straight_right_id = int(getattr(st, "aruco_id_straight_right_pole", 10) or 10)
        straight_left_id  = int(getattr(st, "aruco_id_straight_left_pole", 15) or 15)
        turn_right_id     = int(getattr(st, "aruco_id_turn_right", 20) or 20)
        turn_left_id      = int(getattr(st, "aruco_id_turn_left", 30) or 30)

        def _is_ok_dist(v):
            try:
                d = float(v)
            except Exception:
                return None
            if not (0.05 < d < 50.0):
                return None
            return d

        def _get_id(mi):
            try:
                return int(mi.get("id", -1) or -1)
            except Exception:
                return -1

        def _get_cx(mi):
            try:
                return float(mi.get("cx", 0.5 * W) or (0.5 * W))
            except Exception:
                return float(0.5 * W)

        def _in_last_zone(cx_now: float) -> bool:
            # зона "где был последний раз" (±15% ширины кадра по умолчанию)
            gate_frac = float(getattr(st, "pole_same_gate_frac", 0.15) or 0.15)
            gate_frac = float(_clamp(gate_frac, 0.05, 0.40))
            gate_px = gate_frac * float(W)
            cx0 = getattr(st, "_pole_last_cx", None)
            if cx0 is None:
                return True
            try:
                return abs(float(cx_now) - float(cx0)) <= gate_px
            except Exception:
                return True

        # -----------------------------
        # Normalize input: m can be dict or list[dict] or None
        # -----------------------------
        cand = []
        if m is None:
            cand = []
        elif isinstance(m, (list, tuple)):
            cand = [x for x in m if isinstance(x, dict)]
        elif isinstance(m, dict):
            cand = [m]
        else:
            cand = []

        # -----------------------------
        # Choose one marker
        # -----------------------------
        chosen = None

        # 1) If pole tracking active: prefer straight markers in last zone
        #    BUT only while we are NOT in "losing" phase (loss is sticky and should not be canceled by new markers)
        if bool(getattr(st, "_pole_track_active", False)) and (getattr(st, "_pole_track_lost_ts", None) is None):
            best_d = 1e9
            for mi in cand:
                mid = _get_id(mi)
                if mid not in (straight_left_id, straight_right_id):
                    continue
                cx = _get_cx(mi)
                if not _in_last_zone(cx):
                    continue
                d = _is_ok_dist(mi.get("dist_m", None))
                if d is None:
                    continue
                if d < best_d:
                    best_d = d
                    chosen = mi

        # 2) Turn priority: if we see turn marker and we're locked OR it's very close, choose it
        if chosen is None:
            lock_id = getattr(st, "_turn_lock_id", None)
            best_turn = None
            best_turn_d = 1e9
            for mi in cand:
                mid = _get_id(mi)
                if mid not in (turn_left_id, turn_right_id):
                    continue
                d = _is_ok_dist(mi.get("dist_m", None))
                if d is None:
                    # если нет dist — всё равно можно взять, но хуже
                    d = 1e6
                if lock_id is not None and int(mid) == int(lock_id):
                    best_turn = mi
                    best_turn_d = d
                    break
                if d < best_turn_d:
                    best_turn = mi
                    best_turn_d = d
            # если turn найден и он "достаточно близко" — берём его
            trig_m = float(getattr(st, "marker_turn_trigger_m", 5.0) or 5.0)
            if best_turn is not None and best_turn_d <= float(_clamp(trig_m * 2.0, 0.4, 60.0)):
                chosen = best_turn

        # 3) Default: closest by dist among all allowed IDs (straight/turn/other)
        if chosen is None and cand:
            best_d = 1e9
            best_any = None
            for mi in cand:
                d = _is_ok_dist(mi.get("dist_m", None))
                if d is None:
                    continue
                if d < best_d:
                    best_d = d
                    best_any = mi
            chosen = best_any if best_any is not None else cand[0]

        # -----------------------------
        # Write to state
        # -----------------------------
        if chosen is not None:
            st.marker_seen = True
            st.marker_id = int(chosen.get("id", -1) or -1)
            mid = int(st.marker_id)

            if mid == straight_right_id:
                st.marker_follow_side = "right"
            elif mid == straight_left_id:
                st.marker_follow_side = "left"
            else:
                # не трогаем side, если это не straight
                st.marker_follow_side = getattr(st, "marker_follow_side", None)

            st.marker_dist_m = chosen.get("dist_m", None)
            st.marker_cx = float(chosen.get("cx", 0.5 * W) or (0.5 * W))
            st.marker_cy = float(chosen.get("cy", 0.5 * H) or (0.5 * H))
            st.marker_side = chosen.get("side", None)
            st.marker_ts = now
            st.marker_tvec = chosen.get("tvec", None)
            st.marker_corners = chosen.get("corners", None)
            return

        # --- NO chosen: anti-blink hold ---
        keep_s = float(getattr(st, "marker_keep_s", 0.25) or 0.25)
        last_ts = float(getattr(st, "marker_ts", 0.0) or 0.0)

        if last_ts > 0.0 and (now - last_ts) <= keep_s:
            # HOLD LAST (anti-blink): НЕ трогаем dist/tvec/corners/cx/cy
            st.marker_seen = True
            return

        # реально потеряли
        st.marker_cy = None
        st.marker_seen = False
        st.marker_id = -1
        st.marker_dist_m = None
        st.marker_tvec = None
        st.marker_corners = None
        st.marker_follow_side = None
        st.marker_side = None
        return
    # ============================================================
    # road_bin build & support
    # ============================================================

    def _build_road_bin_from_mask(self, mask: np.ndarray):
        st = self.state
        H, W = mask.shape[:2]

        crop_y0_full = int(H * float(getattr(st, "seg_crop_y0", 0.38) or 0.38))
        crop_y0_full = int(_clamp(crop_y0_full, 0, H - 2))
        mask_crop = mask[crop_y0_full:, :]
        st._seg_crop_y0 = int(crop_y0_full)

        thr = float(getattr(st, "seg_thr", 0.60) or 0.60)
        thr = _clamp(thr, 0.05, 0.95)

        road_bin = (mask_crop > thr)

        # EMA smoothing (cheap)
        ema_alpha = float(getattr(st, "rb_ema_alpha", 0.35) or 0.35)
        ema_alpha = _clamp(ema_alpha, 0.05, 0.85)

        ds = int(getattr(st, "rb_ema_ds", 4) or 4)
        ds = 2 if ds < 2 else 4 if ds > 4 else ds

        rb_u8 = road_bin.astype(np.uint8) * 255
        h0, w0 = rb_u8.shape[:2]
        hs = max(8, h0 // ds)
        ws = max(8, w0 // ds)
        rb_small = cv2.resize(rb_u8, (ws, hs), interpolation=cv2.INTER_AREA).astype(np.float32) / 255.0

        if self._rb_ema_small is None or self._rb_ema_small.shape != rb_small.shape:
            self._rb_ema_small = rb_small
        else:
            self._rb_ema_small = (1.0 - ema_alpha) * self._rb_ema_small + ema_alpha * rb_small

        ema_thr = float(getattr(st, "rb_ema_thr", 0.45) or 0.45)
        ema_thr = _clamp(ema_thr, 0.20, 0.90)

        rb_small_bin = (self._rb_ema_small >= ema_thr).astype(np.uint8) * 255
        rb_smooth_u8 = cv2.resize(rb_small_bin, (w0, h0), interpolation=cv2.INTER_NEAREST)
        road_bin = (rb_smooth_u8 > 0)

        # postprocess (morph + fill holes + keep main component)
        try:
            rb = (road_bin.astype(np.uint8) * 255)

            close_k = int(getattr(st, "rb_close_k", 7) or 7)
            open_k = int(getattr(st, "rb_open_k", 2) or 2)
            close_iter = int(getattr(st, "rb_close_iter", 2) or 2)
            open_iter = int(getattr(st, "rb_open_iter", 1) or 1)

            close_k = int(_clamp(close_k, 3, 21))
            open_k = int(_clamp(open_k, 1, 11))
            close_iter = int(_clamp(close_iter, 1, 5))
            open_iter = int(_clamp(open_iter, 0, 3))

            k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
            k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))

            rb = cv2.morphologyEx(rb, cv2.MORPH_CLOSE, k_close, iterations=close_iter)
            if open_k >= 3 and open_iter > 0:
                rb = cv2.morphologyEx(rb, cv2.MORPH_OPEN, k_open, iterations=open_iter)

            # fill holes
            hh, ww = rb.shape[:2]
            ff = rb.copy()
            mask_ff = np.zeros((hh + 2, ww + 2), np.uint8)
            cv2.floodFill(ff, mask_ff, (0, 0), 255)
            ff_inv = cv2.bitwise_not(ff)
            rb = cv2.bitwise_or(rb, ff_inv)

            # keep main component anchored to bottom band
            num, labels, stats, _ = cv2.connectedComponentsWithStats((rb > 0).astype(np.uint8), connectivity=8)
            if num > 1:
                Hc = rb.shape[0]
                band_h = max(5, int(Hc * float(getattr(st, "rb_keep_bottom_frac", 0.18) or 0.18)))
                bottom = labels[Hc - band_h:, :]

                best_lbl = 0
                best_score = 0
                for lbl in range(1, num):
                    cnt = int(np.sum(bottom == lbl))
                    if cnt > best_score:
                        best_score = cnt
                        best_lbl = lbl

                if best_lbl > 0:
                    rb = ((labels == best_lbl).astype(np.uint8) * 255)

            road_bin = (rb > 0)

        except Exception as e:
            print("[SEG] road_bin postprocess error:", e, flush=True)

        return road_bin, thr, crop_y0_full

    def _compute_road_support(self, st, road_bin: np.ndarray) -> float:
        Hc, Wc = road_bin.shape[:2]
        y0 = int(Hc * float(getattr(st, "road_detect_y0", 0.5) or 0.5))
        y1 = int(Hc * 0.9)
        y0 = int(_clamp(y0, 0, Hc - 2))
        y1 = int(_clamp(y1, y0 + 1, Hc))

        band = road_bin[y0:y1, :]
        road_support = float(band.mean()) if band.size else 0.0

        ema_a = float(getattr(st, "road_support_ema_alpha", 0.25) or 0.25)
        prev = getattr(st, "road_support_ema", None)
        st.road_support_ema = road_support if prev is None else (float(prev) + ema_a * (road_support - float(prev)))

        st.road_detect_band = (y0, y1)
        return float(st.road_support_ema or 0.0)

    # ============================================================
    # Render
    # ============================================================

    def remove_camera_overlay(self):
        sc = self._radar_scene_provider() if self._radar_scene_provider else None
        if sc is None:
            return
        try:
            if self._cam_pixmap_item is not None:
                if self._cam_pixmap_item.scene() is sc:
                    sc.removeItem(self._cam_pixmap_item)
        except Exception:
            pass
        self._cam_pixmap_item = None
        self._cam_fit_once = False
        self._cam_scene_size = None   # <-- КРИТИЧНО: чтобы на возврате камеры sceneRect пересчитался

    def _render_camera(self, frame_bgr, mask=None, road_bin=None, road_support: float = 0.0, thr: float = 0.0):
        if getattr(self.state, "video_mode", "radar") != "camera":
            return

        sc = self._radar_scene_provider() if self._radar_scene_provider else None
        if sc is None:
            return

        vis = frame_bgr

        # road_bin overlay (green)
        if road_bin is not None and getattr(road_bin, "size", 0) > 0:
            try:
                crop_y0 = int(getattr(self.state, "_seg_crop_y0", 0) or 0)
                H, W = frame_bgr.shape[:2]
                full = np.zeros((H, W), dtype=np.uint8)

                rb = (road_bin.astype(np.uint8) * 255)
                hh = min(H - crop_y0, rb.shape[0])
                if hh > 0:
                    if rb.shape[1] == W:
                        full[crop_y0:crop_y0 + hh, :W] = rb[:hh, :W]
                    else:
                        tmp = np.zeros((hh, W), dtype=np.uint8)
                        ww = min(W, rb.shape[1])
                        tmp[:, :ww] = rb[:hh, :ww]
                        full[crop_y0:crop_y0 + hh, :W] = tmp

                color = np.zeros_like(frame_bgr)
                color[:, :, 1] = full
                vis = cv2.addWeighted(frame_bgr, 1.0, color, 0.55, 0.0)
            except Exception:
                vis = frame_bgr

        # main road / rails overlays only if rails_enabled
        if bool(getattr(self.state, "rails_enabled", False)):
            try:
                draw_main_road_overlay(vis, self.state)
            except Exception as e:
                print("[MAIN] overlay error:", e, flush=True)

            try:
                draw_rails_overlay(vis, self.state)
            except Exception as e:
                print("[RAIL] overlay error:", e, flush=True)

        # marker overlay (TTL)
        try:
            st = self.state
            draw_ttl = float(getattr(st, "marker_draw_ttl_s", 0.5) or 0.5)
            ts = float(getattr(st, "marker_ts", 0.0) or 0.0)
            fresh = (ts > 0.0) and ((time.monotonic() - ts) <= draw_ttl)

            if bool(getattr(st, "marker_seen", False)) and fresh:
                corners = getattr(st, "marker_corners", None)
                if corners is not None and len(corners) == 4:
                    pts = np.array(corners, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(vis, [pts], True, (0, 255, 0), 4, cv2.LINE_AA)

                    mid = getattr(st, "marker_id", None)
                    dist = getattr(st, "marker_dist_m", None)
                    dist_txt = "—" if dist is None else f"{float(dist):.2f} m"

                    cx = int(np.mean(pts[:, 0, 0]))
                    cy = int(np.mean(pts[:, 0, 1]))
                    txt = f"ID {mid}  {dist_txt}"

                    cv2.putText(vis, txt, (cx - 10, cy - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 5, cv2.LINE_AA)
                    cv2.putText(vis, txt, (cx - 10, cy - 14),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception:
            pass

        # phone stream push
        try:
            from phone_log_server import phone_frame_update
            phone_frame_update(vis, quality=70)
        except Exception:
            pass

        # to QPixmap safely
        try:
            if vis is None or (not isinstance(vis, np.ndarray)) or vis.ndim != 3 or vis.shape[2] != 3 or vis.size == 0:
                return

            vis = np.ascontiguousarray(vis)
            h0, w0 = vis.shape[:2]
            bpl = int(vis.strides[0])
            exp = 3 * w0
            if bpl != exp:
                vis = np.ascontiguousarray(vis.copy())
                h0, w0 = vis.shape[:2]
                bpl = int(vis.strides[0])
            if not self._scene_noindex_set:
                try:
                    sc.setItemIndexMethod(QtWidgets.QGraphicsScene.NoIndex)
                except Exception:
                    pass
                self._scene_noindex_set = True
            qimg = QtGui.QImage(vis.data, w0, h0, bpl, QtGui.QImage.Format_BGR888).copy()
            pix = QtGui.QPixmap.fromImage(qimg)

            # обновляем sceneRect ТОЛЬКО если размер кадра поменялся
            if self._cam_scene_size != (w0, h0):
                sc.setSceneRect(0, 0, w0, h0)
                self._cam_scene_size = (w0, h0)
                self._cam_fit_once = False

            if self._cam_pixmap_item is None:
                self._cam_pixmap_item = sc.addPixmap(pix)
                self._cam_pixmap_item.setZValue(0)
            else:
                self._cam_pixmap_item.setPixmap(pix)
                self._cam_pixmap_item.setVisible(True)

            views = sc.views() if hasattr(sc, "views") else []
            if views and not self._cam_fit_once:
                try:
                    views[0].fitInView(sc.sceneRect(), QtCore.Qt.KeepAspectRatio)
                    self._cam_fit_once = True
                except Exception:
                    pass

        except Exception as e:
            print("[CAM] render error:", e, flush=True)
            return