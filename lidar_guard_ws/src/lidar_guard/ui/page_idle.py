# page_idle.py
import os
from PyQt5 import QtWidgets, QtCore, QtGui

MODE_MARKERS = "markers"
MODE_ROAD    = "road"
MODE_TRAJ    = "traj"


def _qs() -> QtCore.QSettings:
    return QtCore.QSettings("Rover", "PlatformGUI")


def _as_bool(v, default=False) -> bool:
    if v is None:
        return bool(default)
    if isinstance(v, bool):
        return v
    s = str(v).strip().lower()
    return s in ("1", "true", "yes", "y", "on")


def _load_pixmap(path: str) -> QtGui.QPixmap:
    pm = QtGui.QPixmap(path)
    return pm


class IdlePage(QtCore.QObject):
    """
    Ожидает НОВЫЕ имена из page_idle.ui, который мы сделали:
      - фон: lblIdleBg
      - stack: idleFlowStack (0=start, 1=modes, 2=config)
      - start: btnIdleStart
      - режимы: btnModeMarkers/btnModeRoad/btnModeTraj
      - help: btnHelpMarkers/btnHelpRoad/btnHelpTraj (у них уже toolTip, но мы ещё сделаем overlay help-лейбл)
      - картинки режимов: imgModeMarkers/imgModeRoad/imgModeTraj
      - config: btnChooseMap, lblChosenMapName, btnChooseModel, lblChosenModelName
      - rails: chkRails
      - панели: panelMarkersSettings/panelRoadSettings/panelTrajSettings
      - sliders markers: sldMarkerDist, lblMarkerDistVal, sldMarkerTurnDist, lblMarkerTurnDistVal
      - nav: btnConfigBack, btnConfigNext
    """

    def __init__(self, ui: QtWidgets.QMainWindow, state):
        super().__init__()
        self.ui = ui
        self.state = state
        self.state.in_drive = False
        # ---------- find widgets ----------
        self.pageIdle     = ui.findChild(QtWidgets.QWidget, "pageIdle")
        self.idleFlow     = ui.findChild(QtWidgets.QStackedWidget, "idleFlowStack")
        self.lblBg        = ui.findChild(QtWidgets.QLabel, "lblIdleBg")
        self.overlay      = ui.findChild(QtWidgets.QWidget, "idleOverlay")
        # фон никогда не должен ловить мышь
        if self.lblBg:
            self.lblBg.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        # overlay должен получать клики
        if self.overlay:
            self.overlay.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, False)

        # будем обновлять фон при ресайзе
        if self.pageIdle:
            self.pageIdle.installEventFilter(self)

        # исходный pixmap фона (до масштабирования)
        self._bg_src_pm = None
        self.btnStart     = ui.findChild(QtWidgets.QPushButton, "btnIdleStart")

        self.btnModeMarkers = ui.findChild(QtWidgets.QPushButton, "btnModeMarkers")
        self.btnModeRoad    = ui.findChild(QtWidgets.QPushButton, "btnModeRoad")
        self.btnModeTraj    = ui.findChild(QtWidgets.QPushButton, "btnModeTraj")

        self.btnHelpMarkers = ui.findChild(QtWidgets.QToolButton, "btnHelpMarkers")
        self.btnHelpRoad    = ui.findChild(QtWidgets.QToolButton, "btnHelpRoad")
        self.btnHelpTraj    = ui.findChild(QtWidgets.QToolButton, "btnHelpTraj")

        self.imgModeMarkers = ui.findChild(QtWidgets.QLabel, "imgModeMarkers")
        self.imgModeRoad    = ui.findChild(QtWidgets.QLabel, "imgModeRoad")
        self.imgModeTraj    = ui.findChild(QtWidgets.QLabel, "imgModeTraj")

        # config widgets
        self.btnChooseMap   = ui.findChild(QtWidgets.QPushButton, "btnChooseMap")
        self.lblChosenMap   = ui.findChild(QtWidgets.QLabel, "lblChosenMapName")

        self.btnChooseModel = ui.findChild(QtWidgets.QPushButton, "btnChooseModel")
        self.lblChosenModel = ui.findChild(QtWidgets.QLabel, "lblChosenModelName")

        self.chkRails       = ui.findChild(QtWidgets.QCheckBox, "chkRails")

        self.panelMarkers   = ui.findChild(QtWidgets.QWidget, "panelMarkersSettings")
        self.panelRoad      = ui.findChild(QtWidgets.QWidget, "panelRoadSettings")
        self.panelTraj      = ui.findChild(QtWidgets.QWidget, "panelTrajSettings")

        self.sldMarkerDist      = ui.findChild(QtWidgets.QSlider, "sldMarkerDist")
        self.lblMarkerDistVal   = ui.findChild(QtWidgets.QLabel, "lblMarkerDistVal")
        self.sldMarkerTurnDist  = ui.findChild(QtWidgets.QSlider, "sldMarkerTurnDist")
        self.lblMarkerTurnVal   = ui.findChild(QtWidgets.QLabel, "lblMarkerTurnDistVal")
        self.sldPoleSpacing     = ui.findChild(QtWidgets.QSlider, "sldPoleSpacing")
        self.lblPoleSpacingVal  = ui.findChild(QtWidgets.QLabel,  "lblPoleSpacingVal")
        self.btnBack = ui.findChild(QtWidgets.QPushButton, "btnConfigBack")
        self.btnNext = ui.findChild(QtWidgets.QPushButton, "btnConfigNext")

        # ---------- safety: critical widgets ----------
        if not self.idleFlow:
            raise RuntimeError("UI mismatch: can't find QStackedWidget 'idleFlowStack' in page_idle.ui")
        if not self.lblBg:
            raise RuntimeError("UI mismatch: can't find QLabel 'lblIdleBg' in page_idle.ui")

        # ---------- blur effect ----------
        self._blur = QtWidgets.QGraphicsBlurEffect()
        self._blur.setBlurRadius(14)

        # ---------- overlay help label (если в UI нет отдельного lblModeHelp) ----------
        self.lblModeHelp = ui.findChild(QtWidgets.QLabel, "lblModeHelp")
        if not self.lblModeHelp and self.overlay:
            self.lblModeHelp = QtWidgets.QLabel(self.overlay)
            self.lblModeHelp.setObjectName("lblModeHelp")
            self.lblModeHelp.setWordWrap(True)
            self.lblModeHelp.setAlignment(QtCore.Qt.AlignCenter)
            self.lblModeHelp.setStyleSheet(
                "background: rgba(0,0,0,180); color: white; "
                "border-radius: 16px; padding: 14px 18px;"
            )
            self.lblModeHelp.hide()
        self.lblModeHelp.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        self._help_text = {
            MODE_MARKERS: "Ровер держит выбранную дистанцию до ориентиров и корректирует курс по ним. Подходит для движения в коридоре/между объектами.",
            MODE_ROAD:    "Ровер старается держаться в пределах дороги и плавно проходит повороты. Подходит, когда есть понятная полоса движения.",
            MODE_TRAJ:    "Ровер следует заранее заданному маршруту и старается не отклоняться от траектории. Подходит для повторяемых поездок.",
        }

        # ---------- cache base styles for red highlighting ----------
        self._base_btn_styles = {}
        for w in (self.btnChooseMap, self.btnChooseModel):
            if w:
                self._base_btn_styles[w.objectName()] = w.styleSheet() or ""

        # ---------- wiring ----------
        if self.btnStart:
            self.btnStart.clicked.connect(self._go_mode_select)

        if self.btnModeMarkers:
            self.btnModeMarkers.clicked.connect(lambda: self._select_mode(MODE_MARKERS))
        if self.btnModeRoad:
            self.btnModeRoad.clicked.connect(lambda: self._select_mode(MODE_ROAD))
        if self.btnModeTraj:
            self.btnModeTraj.clicked.connect(lambda: self._select_mode(MODE_TRAJ))

        for btn, mode in (
            (self.btnHelpMarkers, MODE_MARKERS),
            (self.btnHelpRoad,    MODE_ROAD),
            (self.btnHelpTraj,    MODE_TRAJ),
        ):
            if btn:
                btn.setProperty("modeHelp", mode)
                btn.clicked.connect(lambda _=False, m=mode: self._toggle_help(m))

        if self.btnChooseMap:
            self.btnChooseMap.clicked.connect(self._choose_map)

        if self.btnChooseModel:
            self.btnChooseModel.clicked.connect(self._choose_model)

        if self.sldMarkerDist:
            self.sldMarkerDist.setTracking(True)
            self.sldMarkerDist.valueChanged.connect(self._on_marker_dist)
        if self.sldMarkerTurnDist:
            self.sldMarkerTurnDist.setTracking(True)
            self.sldMarkerTurnDist.valueChanged.connect(self._on_marker_turn_dist)
        if self.sldPoleSpacing:
            self.sldPoleSpacing.setTracking(True)
            self.sldPoleSpacing.valueChanged.connect(self._on_pole_spacing)
        if self.chkRails:
            self.chkRails.toggled.connect(self._on_rails)

        if self.btnBack:
            self.btnBack.clicked.connect(self._back)
        if self.btnNext:
            self.btnNext.clicked.connect(self._next)

        # ---------- load persistent ----------
        self._load_idle_settings()
        self._apply_mode_ui()
        self._sync_labels()
        # ---------- load images into UI (фон + 3 режима) ----------
        self._load_idle_images()
        if self.lblBg:
            self.lblBg.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
        if self.overlay:
            self.overlay.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, False)        
        self._fix_z_order()
        self.idleFlow.setCurrentIndex(0)

    # -------------------- images --------------------
    def _load_idle_images(self):
        """
        Строго грузим из папки ./idle_images:
          idle_bg.png
          mode_markers.png
          mode_road.png
          mode_traj.png
        """
        # 1) сначала пробуем относительно cwd (как ты запускаешь проект)
        img_dir = os.path.abspath(os.path.join(os.getcwd(), "idle_images"))

        # 2) fallback: рядом с page_idle.py
        if not os.path.isdir(img_dir):
            img_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "idle_images"))

        if not os.path.isdir(img_dir):
            print(f"[IdlePage] idle_images folder not found: {img_dir}")
            return

        bg_path   = os.path.join(img_dir, "idle_bg.png")
        m_path    = os.path.join(img_dir, "mode_markers.png")
        road_path = os.path.join(img_dir, "mode_road.png")
        traj_path = os.path.join(img_dir, "mode_traj.png")

        def set_pm(lbl: QtWidgets.QLabel, path: str, is_bg: bool = False):
            if not lbl:
                return
            if not os.path.exists(path):
                print(f"[IdlePage] image missing: {path}")
                return

            pm = _load_pixmap(path)
            if pm.isNull():
                print(f"[IdlePage] failed to load: {path}")
                return

            if is_bg:
                self._bg_src_pm = pm
                self._apply_bg_pixmap()
            else:
                lbl.setPixmap(pm)

        set_pm(self.lblBg, bg_path, is_bg=True)
        set_pm(self.imgModeMarkers, m_path)
        set_pm(self.imgModeRoad, road_path)
        set_pm(self.imgModeTraj, traj_path)

        print("[IdlePage] images OK:",
              "\n  bg   =", bg_path,
              "\n  mark =", m_path,
              "\n  road =", road_path,
              "\n  traj =", traj_path)
        self._fix_z_order()
    def eventFilter(self, obj, ev):
        if obj is self.pageIdle and ev.type() == QtCore.QEvent.Resize:
            self._apply_bg_pixmap()
        return super().eventFilter(obj, ev)

    def _apply_bg_pixmap(self):
        """Масштабирует фон минимум до 1920x1280 и под текущий размер QLabel."""
        if not self.lblBg or self._bg_src_pm is None or self._bg_src_pm.isNull():
            return

        w = max(1, self.lblBg.width())
        h = max(1, self.lblBg.height())

        pm = self._bg_src_pm.scaled(
            w, h,
            QtCore.Qt.KeepAspectRatioByExpanding,
            QtCore.Qt.SmoothTransformation
        )
        self.lblBg.setPixmap(pm)

    def _fix_z_order(self):
        """Гарантирует, что фон снизу, а overlay + кнопки сверху."""
        if self.lblBg:
            self.lblBg.lower()
        if self.overlay:
            self.overlay.raise_()
        if self.idleFlow:
            self.idleFlow.raise_()
    # -------------------- hover help --------------------

    def _show_help(self, mode: str):
        if not self.lblModeHelp or not self.overlay:
            return
        txt = self._help_text.get(mode, "")
        if not txt:
            return

        self.lblModeHelp.setText(txt)
        # позиционируем по центру, ближе к низу, но не перекрываем навигацию
        W = self.overlay.width()
        H = self.overlay.height()
        w = int(W * 0.70)
        h = 120
        x = (W - w) // 2
        y = int(H * 0.72)
        self.lblModeHelp.setGeometry(x, y, w, h)

        self.lblModeHelp.show()
        self.lblModeHelp.raise_()

    def _hide_help(self):
        if self.lblModeHelp:
            self.lblModeHelp.hide()
    def _toggle_help(self, mode: str):
        if not self.lblModeHelp or not self.overlay:
            return
        if self.lblModeHelp.isVisible():
            self._hide_help()
        else:
            self._show_help(mode)
    # -------------------- wizard flow --------------------
    def _go_mode_select(self):
        # включаем blur фона
        if self.lblBg:
            self.lblBg.setGraphicsEffect(self._blur)
        self.idleFlow.setCurrentIndex(1)

    def _select_mode(self, mode: str):
        self.state.nav_mode = mode
        self._save_idle_settings()
        self._apply_mode_ui()
        self.idleFlow.setCurrentIndex(2)

    def _back(self):
        idx = self.idleFlow.currentIndex()
        if idx == 2:
            self.idleFlow.setCurrentIndex(1)
        elif idx == 1:
            self.idleFlow.setCurrentIndex(0)
            if self.lblBg:
                self.lblBg.setGraphicsEffect(None)

    def _next(self):
        ok = True

        # карта обязательна всегда
        if not getattr(self.state, "active_map_path", None):
            ok = False
            self._mark_required(self.btnChooseMap, True)
        else:
            self._mark_required(self.btnChooseMap, False)

        # модель обязательна только для road
        if getattr(self.state, "nav_mode", MODE_MARKERS) == MODE_ROAD:
            if not getattr(self.state, "road_model_path", None):
                ok = False
                self._mark_required(self.btnChooseModel, True)
            else:
                self._mark_required(self.btnChooseModel, False)
        else:
            self._mark_required(self.btnChooseModel, False)

        if not ok:
            return

        # всё ок -> в DRIVE
        stack = self.ui.findChild(QtWidgets.QStackedWidget, "stackRoot")
        pageDrive = self.ui.findChild(QtWidgets.QWidget, "pageDrive")
        if stack and pageDrive:
            self.state.in_drive = True          # <-- ВОТ ТУТ
            stack.setCurrentWidget(pageDrive)

    def _mark_required(self, btn: QtWidgets.QWidget, on: bool):
        if not btn:
            return
        base = self._base_btn_styles.get(btn.objectName(), btn.styleSheet() or "")
        red = "border:2px solid #d64545; border-radius:12px;"
        if on:
            btn.setStyleSheet(base + "\n" + red)
        else:
            btn.setStyleSheet(base)

    # -------------------- choose map/model --------------------
    def _choose_map(self):
        """
        По умолчанию дергаем твой текущий механизм (как ты делал раньше).
        Если его нет — дадим fallback QFileDialog.
        """
        if hasattr(self.ui, "pick_map_and_load"):
            self.ui.pick_map_and_load()
        else:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self.ui, "Выберите PNG карту", "", "Images (*.png *.jpg *.jpeg)"
            )
            if path:
                self.state.active_map_path = path

        self._save_idle_settings()
        self._sync_labels()
    def _maps_repo_dir(self) -> str:
        # page_idle.py лежит в ui/ => maps_repo рядом: ui/maps_repo
        here = os.path.dirname(__file__)
        cand = os.path.abspath(os.path.join(here, "maps_repo"))
        if os.path.isdir(cand):
            return cand
        return os.getcwd()
    def _choose_model(self):
        start_dir = self._maps_repo_dir()
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self.ui,
            "Выберите модель (.onnx)",
            start_dir,
            "ONNX (*.onnx)"
        )
        if path:
            self.state.road_model_path = path
            self._save_idle_settings()
            self._sync_labels()
    # -------------------- mode ui --------------------
    def _apply_mode_ui(self):
        mode = getattr(self.state, "nav_mode", MODE_MARKERS)

        if self.panelMarkers:
            self.panelMarkers.setVisible(mode == MODE_MARKERS)
        if self.panelRoad:
            self.panelRoad.setVisible(mode == MODE_ROAD)
        if self.panelTraj:
            self.panelTraj.setVisible(mode == MODE_TRAJ)

        # model widgets only for road
        if self.btnChooseModel:
            self.btnChooseModel.setVisible(mode == MODE_ROAD)
        if self.lblChosenModel:
            self.lblChosenModel.setVisible(mode == MODE_ROAD)
        if self.chkRails:
            self.chkRails.setVisible(mode == MODE_ROAD)

    def _sync_labels(self):
        if self.lblChosenMap:
            mp = getattr(self.state, "active_map_path", None)
            self.lblChosenMap.setText(os.path.basename(mp) if mp else "—")

        if self.lblChosenModel:
            mp = getattr(self.state, "road_model_path", None)
            self.lblChosenModel.setText(os.path.basename(mp) if mp else "—")
        if self.lblMarkerDistVal and self.sldMarkerDist:
            self.lblMarkerDistVal.setText(f"{self.state.marker_target_side_m:.1f}")
        if self.lblMarkerTurnVal and self.sldMarkerTurnDist:
            self.lblMarkerTurnVal.setText(str(int(self.sldMarkerTurnDist.value())))
        if self.lblPoleSpacingVal and self.sldPoleSpacing:
            self.lblPoleSpacingVal.setText(str(int(self.sldPoleSpacing.value())))
    # -------------------- settings handlers --------------------
    def _on_marker_dist(self, v: int):
        dist_m = float(v) * 0.5
        dist_m = max(0.5, min(5.0, dist_m))

        print("[IdlePage] marker_target_side_m =", dist_m)

        self.state.marker_target_side_m = dist_m
        self._save_idle_settings()

        if self.lblMarkerDistVal:
            self.lblMarkerDistVal.setText(f"{dist_m:.1f}")

    def _on_marker_turn_dist(self, v: int):
        print("[IdlePage] marker_turn_dist =", v)
        self.state.marker_turn_trigger_m = float(v)
        self._save_idle_settings()
        self._sync_labels()
    def _on_pole_spacing(self, v: int):
        v = int(v)
        v = max(20, min(35, v))
        print("[IdlePage] pole_spacing_m =", v)
        self.state.pole_spacing_m = v
        self._save_idle_settings()
        self._sync_labels()
    def _on_rails(self, on: bool):
        self.state.rails_enabled = bool(on)
        self._save_idle_settings()

    # -------------------- persistence --------------------
    def _load_idle_settings(self):
        s = _qs()

        self.state.nav_mode = s.value("idle/nav_mode", MODE_MARKERS) or MODE_MARKERS

        # карта/модель (мы это тоже сохраняем, чтобы после рестарта подтянулось)
        self.state.active_map_path  = s.value("idle/active_map_path", "") or ""
        self.state.road_model_path  = s.value("road/model_path", "") or ""

        self.state.rails_enabled = _as_bool(s.value("road/rails_enabled", "0"), default=False)

        self.state.marker_target_side_m = float(
            s.value("markers/marker_target_side_m", 2.0)
        )
        self.state.marker_turn_trigger_m = float(
            s.value("markers/marker_turn_trigger_m", 5.0)
)
        self.state.pole_spacing_m = int(
            s.value("markers/pole_spacing_m", 25)
        )
        # применим в UI
        if self.chkRails:
            self.chkRails.setChecked(bool(self.state.rails_enabled))

        if self.sldMarkerDist:
            self.sldMarkerDist.setMinimum(1)    # 0.5 м
            self.sldMarkerDist.setMaximum(10)   # 5.0 м
            self.sldMarkerDist.setSingleStep(1) # шаг = 0.5
            self.sldMarkerDist.setValue(
                int(round(float(getattr(self.state, "marker_target_side_m", 2.0)) * 2))
            )
        if self.sldMarkerTurnDist:
            self.sldMarkerTurnDist.setMinimum(1)
            self.sldMarkerTurnDist.setMaximum(8)
            self.sldMarkerTurnDist.setValue(
                int(getattr(self.state, "marker_turn_trigger_m", 5.0))
            )

        if self.sldPoleSpacing:
            self.sldPoleSpacing.setMinimum(20)
            self.sldPoleSpacing.setMaximum(35)
            self.sldPoleSpacing.setValue(
                int(getattr(self.state, "pole_spacing_m", 25))
            )

    def _save_idle_settings(self):
        s = _qs()

        s.setValue("idle/nav_mode", getattr(self.state, "nav_mode", MODE_MARKERS))
        s.setValue("idle/active_map_path", getattr(self.state, "active_map_path", "") or "")

        s.setValue("road/rails_enabled", "1" if getattr(self.state, "rails_enabled", False) else "0")
        s.setValue("road/model_path", getattr(self.state, "road_model_path", "") or "")
        s.setValue(
            "markers/marker_target_side_m",
            float(getattr(self.state, "marker_target_side_m", 2.0) or 2.0),
        )
        s.setValue(
            "markers/marker_turn_trigger_m",
            float(getattr(self.state, "marker_turn_trigger_m", 5.0) or 5.0),
        )
        s.setValue(
            "markers/pole_spacing_m",
            int(getattr(self.state, "pole_spacing_m", 25) or 25),
        )