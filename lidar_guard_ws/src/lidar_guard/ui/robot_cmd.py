# robot_cmd.py
# -*- coding: utf-8 -*-
import os
import glob
import time
import math
from typing import Optional, Tuple
from hw_ports import get_arduino_port
from PyQt5 import QtWidgets, QtCore

# ================== ПАРАМЕТРЫ СЕРВО-ДРАЙВА ==================
PWM_MIN = 1000          # минимальный импульс (мкс)
PWM_NEUTRAL = 1500      # нейтраль
PWM_MAX = 2000          # максимум

# Максимальная линейная скорость при L=R=PWM_MAX (для HUD)
MAX_SPEED_MPS = 5
_prev_lrb = None
# ================== СЕРИАЛ К ARDUINO ==================
try:
    import serial  # pyserial
except Exception:
    serial = None

_SER = None
_SER_PORT: Optional[str] = None
_LAST_SENT: Tuple[Optional[int], Optional[int], Optional[int]] = (None, None, None)
_LAST_FAIL_TS = 0.0


def _clamp_servo_us(x: Optional[int]) -> int:
    """
    Клэмп к диапазону [PWM_MIN, PWM_MAX].
    None -> нейтраль.
    """
    if x is None:
        return PWM_NEUTRAL
    try:
        xi = int(x)
    except Exception:
        xi = PWM_NEUTRAL
    if xi < PWM_MIN:
        return PWM_MIN
    if xi > PWM_MAX:
        return PWM_MAX
    return xi


def _pick_port() -> Optional[str]:

    return get_arduino_port()


def _ensure_serial() -> bool:
    """
    Открываем порт, если ещё не открыт.
    """
    global _SER, _SER_PORT, _LAST_FAIL_TS

    if serial is None:
        if time.time() - _LAST_FAIL_TS > 3.0:
            print("[ARDUINO] pyserial не установлен — команды не отправляются", flush=True)
            _LAST_FAIL_TS = time.time()
        return False

    if _SER and _SER.is_open:
        return True

    if time.time() - _LAST_FAIL_TS < 0.8:
        return False

    port = _pick_port()
    if not port:
        if time.time() - _LAST_FAIL_TS > 3.0:
            print("[ARDUINO] последовательный порт не найден — пропускаю отправку", flush=True)
            _LAST_FAIL_TS = time.time()
        return False

    try:
        _SER = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.02,
            write_timeout=0.05,
        )
        _SER_PORT = port
        print(f"[ARDUINO] открыт порт {port}", flush=True)
        return True
    except Exception as e:
        if time.time() - _LAST_FAIL_TS > 3.0:
            print(f"[ARDUINO] не удалось открыть порт {port}: {e}", flush=True)
            _LAST_FAIL_TS = time.time()
        _SER = None
        _SER_PORT = None
        return False


def _send_m(l_us: int, r_us: int, b_us: int) -> None:
    """
    Отправка команды на Arduino в СЕРВО-ФОРМАТЕ:
      M L=<1000..2000> R=<1000..2000> B=<1000..2000>\\n
    """
    global _prev_lrb

    l = _clamp_servo_us(l_us)
    r = _clamp_servo_us(r_us)
    b = _clamp_servo_us(b_us)
    triple = (l, r, b)
    if triple != _prev_lrb:
        print(f"LER L={l} R={r} B={b}", flush=True)
        _prev_lrb = triple
    if not _ensure_serial():
        return

    msg = f"M L={l} R={r} B={b}\n"
    try:
        _SER.write(msg.encode("ascii", errors="ignore"))
        _LAST_SENT = (l, r, b)
    except Exception as e:
        print(f"[ARDUINO] ошибка отправки: {e}", flush=True)


# ================== МОТОРЫ / ИНСТРУМЕНТ ==================
def motors_set(state, l_pwm, r_pwm, b_pwm=None):
    """
    Отправка PWM на моторы + логирование. Лог не спамит,
    если значения не поменялись.
    """
    global _prev_lrb

    # нормализуем значения
    l = int(l_pwm or 0)
    r = int(r_pwm or 0)
    b = int(b_pwm or 0) if b_pwm is not None else 0

    triple = (l, r, b)

    # логируем только при изменении
    if triple != _prev_lrb:
        print(f"LER L={l} R={r} B={b}", flush=True)
        _prev_lrb = triple

    # дальше — твоя существующая логика отправки в Arduino/серво:
    #   - запись в state
    #   - отправка по UART / ROS
    state.manual_l_pwm = l
    state.manual_r_pwm = r
    state.manual_b_pwm = b



def motors_stop(state) -> None:
    """
    Полная остановка: все каналы в нейтраль (1500).
    """
    state.motor_L = PWM_NEUTRAL
    state.motor_R = PWM_NEUTRAL
    state.motor_B = PWM_NEUTRAL
    _send_m(PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL)
    state.speed_mps = 0.0


def tool_pulse(ui: QtWidgets.QMainWindow,
               state,
               level_us: int = PWM_MAX,
               ms: int = 200) -> None:
    """
    Краткий импульс по вспомогательному каналу B:
      - на ms миллисекунд ставим B = level_us
      - потом возвращаем прежний B.

    Используется, например, при прохождении контрольной точки,
    чтобы дёрнуть реле / зуммер / что угодно.
    """
    base_L = int(getattr(state, "motor_L", PWM_NEUTRAL) or PWM_NEUTRAL)
    base_R = int(getattr(state, "motor_R", PWM_NEUTRAL) or PWM_NEUTRAL)
    base_B = int(getattr(state, "motor_B", PWM_NEUTRAL) or PWM_NEUTRAL)

    # Вперёд как есть, только B поднимаем
    motors_set(state, base_L, base_R, level_us)

    def _off():
        motors_set(state, base_L, base_R, base_B)

    QtCore.QTimer.singleShot(max(1, int(ms)), _off)


# ================== СКОРОСТЬ ДЛЯ HUD ==================
def get_speed(state) -> float:
    """
    Оцениваем линейную скорость из текущих PWM моторов.

    Принимаем линейную зависимость:
      - PWM_NEUTRAL (1500) -> 0 м/с
      - PWM_MAX     (2000) -> +MAX_SPEED_MPS
      - PWM_MIN     (1000) -> -MAX_SPEED_MPS
    """
    l = float(getattr(state, "manual_l_pwm", PWM_NEUTRAL) or PWM_NEUTRAL)
    r = float(getattr(state, "manual_r_pwm", PWM_NEUTRAL) or PWM_NEUTRAL)
    avg = (l + r) / 2.0

    # От 1500 до 2000 -> 0..+1
    if avg >= PWM_NEUTRAL:
        if PWM_MAX == PWM_NEUTRAL:
            frac = 0.0
        else:
            frac = (avg - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL)
    # От 1000 до 1500 -> -1..0
    else:
        if PWM_NEUTRAL == PWM_MIN:
            frac = 0.0
        else:
            frac = -(PWM_NEUTRAL - avg) / (PWM_NEUTRAL - PWM_MIN)

    # Клэмп на всякий
    if frac > 1.0:
        frac = 1.0
    if frac < -1.0:
        frac = -1.0
    state.speed_mps = MAX_SPEED_MPS * frac
    return MAX_SPEED_MPS * frac


def set_guard_state(state, value: str):
    state.guard_state = str(value)


def set_safety_stop(ui: QtWidgets.QMainWindow, enabled: bool, state=None):
    w = ui.findChild(QtWidgets.QLabel, "lblSafetyStop")
    if w:
        if enabled:
            w.setStyleSheet(
                "color:#ffffff; background:#d64545; padding:2px 6px; border-radius:4px;"
            )
            w.setText("ПРЕПЯТСТВИЕ")
        else:
            w.setStyleSheet("")
            w.setText("")
    if enabled and state is not None:
        motors_stop(state)


# ================== ПАМЯТЬ ПОЗИЦИИ РОБОТА ==================
def note_robot_pose(state, min_move_px: float = 3.0) -> None:
    rp = getattr(state, "robot_px", None)
    if not rp:
        return

    x, y = float(rp[0]), float(rp[1])
    last = getattr(state, "last_robot_px", None)

    if last is None:
        state.last_robot_px = (x, y)
        return

    lx, ly = float(last[0]), float(last[1])
    dx = x - lx
    dy = y - ly
    d2 = dx * dx + dy * dy

    if d2 >= (min_move_px * min_move_px):
        state.last_robot_px_prev = (lx, ly)
        state.last_robot_px = (x, y)
    else:
        state.last_robot_px = (x, y)


# ================== ЛОГИКА «АВТОРАЗВОРОТ НА СТАРТЕ» ==================

def _route_forward_dir_px(state, lookahead_px: float = 40.0) -> Optional[tuple[float, float]]:
    """
    Приблизительное направление маршрута вперёд от начала (в пикселях).
    Идём от route_pts_px[0] по маршруту, пока не набежит lookahead_px.
    """
    pts = getattr(state, "route_pts_px", None) or []
    if len(pts) < 2:
        return None

    x0, y0 = pts[0]
    acc = 0.0
    for j in range(1, len(pts)):
        x1, y1 = pts[j]
        dx = x1 - x0
        dy = y1 - y0
        ds = math.hypot(dx, dy)
        acc += ds
        if acc >= lookahead_px:
            return (dx, dy)
        x0, y0 = x1, y1

    # маршрут короткий — берём последний отрезок
    x1, y1 = pts[-1]
    dx = x1 - pts[0][0]
    dy = y1 - pts[0][1]
    if math.hypot(dx, dy) < 1e-3:
        return None
    return (dx, dy)


def _motion_dir_px(state, min_move_px: float = 2.0) -> Optional[tuple[float, float]]:
    """
    Направление последнего движения робота по траектории last_robot_px_prev -> last_robot_px.
    """
    last = getattr(state, "last_robot_px", None)
    prev = getattr(state, "last_robot_px_prev", None)
    if not last or not prev:
        return None

    lx, ly = float(last[0]), float(last[1])
    px, py = float(prev[0]), float(prev[1])
    dx = lx - px
    dy = ly - py
    if dx * dx + dy * dy < (min_move_px * min_move_px):
        return None
    return (dx, dy)


def _angle_between(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    """
    Угол между векторами в градусах [0..180].
    """
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    dot = (x1 * x2 + y1 * y2) / (n1 * n2)
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def _need_autoturn_on_start(state,
                            angle_thresh_deg: float = 120.0,
                            min_move_px: float = 2.0) -> bool:
    """
    Решаем, надо ли разворачиваться на старте:
      - если направление движения робота почти противоположно
        направлению маршрута на старте, делаем разворот.
    """
    v_route = _route_forward_dir_px(state, lookahead_px=40.0)
    v_move  = _motion_dir_px(state, min_move_px=min_move_px)

    if v_route is None or v_move is None:
        print("[TURN DBG] no route/motion dir → no autoturn", flush=True)
        return False

    angle = _angle_between(v_move, v_route)
    print(f"[TURN DBG] start angle between motion and route = {angle:.1f}°", flush=True)
    return angle >= angle_thresh_deg


def _turn_right_only(state, pwm: int = 200) -> None:
    """
    Поворот вправо: правая гусеница вперёд, левая назад.
    pwm — насколько отклоняемся от нейтрали (1500 ± pwm).
    """
    delta = int(pwm)
    base_L = PWM_NEUTRAL - delta
    base_R = PWM_NEUTRAL + delta
    motors_set(state, base_L, base_R, None)


def autoturn_on_route_start(state, pwm: int = 200) -> bool:
    """
    Вызываем при нажатии ПУСК.
    Если робот ехал в сторону, противоположную маршруту, — разворачиваемся.
    Иначе — стартуем как есть.
    """
    rpx  = getattr(state, "robot_px", None)
    last = getattr(state, "last_robot_px", None)
    prev = getattr(state, "last_robot_px_prev", None)
    pts  = getattr(state, "route_pts_px", None) or []
    print(
        f"[TURN DBG] autoturn_on_route_start: robot_px={rpx}, "
        f"last={last}, prev={prev}, route_pts={len(pts)}",
        flush=True,
    )

    if _need_autoturn_on_start(state, angle_thresh_deg=120.0):
        print("[TURN] автоворот на месте (направление движения противоположно маршруту)", flush=True)
        state.turn_mode = "waiting_lost"
        state.turn_pending_route_start = True
        state._turn_prev_road_state = getattr(state, "road_state", "unknown")
        _turn_right_only(state, pwm=pwm)
        return False
    # обычный старт без разворота
    state.turn_mode = "off"
    state.turn_pending_route_start = False
    state._turn_prev_road_state = getattr(state, "road_state", "unknown")
    return True

def apply_start_defaults(state) -> None:
    """
    Стартовое значение вперёд для автономного режима.
    Берём из state.drive_pwm_us, если задано, иначе 1700.
    """
    start_us = int(getattr(state, "drive_pwm_us", 1700) or 1700)
    motors_set(state, start_us, start_us, PWM_NEUTRAL)


# ================== HUD ==================
from routing import route_caption_text  # используем версию из routing.py


def update_drive_panel(ui: QtWidgets.QMainWindow, state) -> None:
    """
    Обновляет:
      - lblMinDist (многострочный статус маршрута)
      - lblSpeed   (оценка скорости по моторам)
    """
    lbl = ui.findChild(QtWidgets.QLabel, "lblMinDist")
    if lbl and not getattr(lbl, "_init_multiline", False):
        lbl.setTextFormat(QtCore.Qt.PlainText)
        lbl.setWordWrap(True)
        lbl.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        sp = lbl.sizePolicy()
        sp.setVerticalPolicy(QtWidgets.QSizePolicy.MinimumExpanding)
        lbl.setSizePolicy(sp)
        lbl.setMinimumHeight(64)
        lbl._init_multiline = True

    if lbl:
        try:
            lbl.setText(route_caption_text(state))
        except Exception as e:
            print("[HUD] route_caption_text error:", e, flush=True)

    ws = ui.findChild(QtWidgets.QLabel, "lblSpeed")
    if ws:
        try:
            ws.setText(f"{get_speed(state):.1f} м/с")
        except Exception:
            ws.setText("0.0 м/с")