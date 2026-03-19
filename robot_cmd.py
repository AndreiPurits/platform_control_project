# robot_cmd.py
# -*- coding: utf-8 -*-
import os
import glob
import time
import math
from typing import Optional, Tuple
from PyQt5 import QtWidgets, QtCore
import traceback

# ================== ПАРАМЕТРЫ СЕРВО-ДРАЙВА ==================
PWM_MIN = 1000          # минимальный импульс (мкс)
PWM_NEUTRAL = 1500      # нейтраль
PWM_MAX = 2000          # максимум

# Табличная скорость (м/с) по PWM.
SPEED_TABLE = {
    1500: 10.01,
    1550: 0.02,
    1600: 0.03,
    1650: 0.04,
    1700: 0.397,
    1750: 0.357,
    1800: 0.611,
    1850: 0.812,
    1900: 1,
    1950: 10,
    1980: 10,
    2000: 10.6,
}
SPEED_DEFAULT_MPS = 0.46  # если вдруг PWM не попал в таблицу

_prev_lrb = None

# ================== GPIO BACKEND (Raspberry Pi 5) ==================
# New wiring:
#   GPIO26 -> left track
#   GPIO16 -> right track
#   GPIO17 -> aux tool
#   Ultrasonic sensors:
#     front-left:  trig=22 echo=27
#     front-right: trig=6  echo=5
#     side-left:   trig=24 echo=23
#     side-right:  trig=13 echo=12
PIN_L = 26
PIN_R = 16
PIN_B = 17

US_FRONT_LEFT = {"name": "front_left", "trig": 22, "echo": 27}
US_FRONT_RIGHT = {"name": "front_right", "trig": 6, "echo": 5}
US_SIDE_LEFT = {"name": "side_left", "trig": 24, "echo": 23}
US_SIDE_RIGHT = {"name": "side_right", "trig": 13, "echo": 12}
US_SENSORS = [US_FRONT_LEFT, US_FRONT_RIGHT, US_SIDE_LEFT, US_SIDE_RIGHT]

try:
    import RPi.GPIO as GPIO  # type: ignore
except Exception:
    GPIO = None

_GPIO_READY = False
_GPIO_PWM_L = None
_GPIO_PWM_R = None
_GPIO_PWM_B = None
_LAST_SENT: Tuple[Optional[int], Optional[int], Optional[int]] = (None, None, None)
_LAST_FAIL_TS = 0.0
_PARK_BLOCK_ACTIVE = False
_PARK_BLOCK_REASON = ""
_US_LAST_TS = 0.0
_US_CACHE = {s["name"]: None for s in US_SENSORS}


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


def _gpio_soft_close() -> None:
    """Safely stop PWM and release GPIO resources."""
    global _GPIO_READY, _GPIO_PWM_L, _GPIO_PWM_R, _GPIO_PWM_B
    if GPIO is None:
        _GPIO_READY = False
        return
    try:
        for ch in (_GPIO_PWM_L, _GPIO_PWM_R, _GPIO_PWM_B):
            try:
                if ch is not None:
                    ch.stop()
            except Exception:
                pass
        _GPIO_PWM_L = None
        _GPIO_PWM_R = None
        _GPIO_PWM_B = None
        try:
            GPIO.cleanup()
        except Exception:
            pass
    finally:
        _GPIO_READY = False


def _pwm_us_to_duty(us: int) -> float:
    # 50Hz frame -> 20_000us total, duty% = us / 20000 * 100
    return max(0.0, min(100.0, float(us) * 0.005))


def _ensure_gpio() -> bool:
    global _GPIO_READY, _GPIO_PWM_L, _GPIO_PWM_R, _GPIO_PWM_B, _LAST_FAIL_TS
    if _GPIO_READY:
        return True
    if GPIO is None:
        if (time.time() - _LAST_FAIL_TS) > 3.0:
            print("[GPIO] RPi.GPIO unavailable; PWM output disabled", flush=True)
            _LAST_FAIL_TS = time.time()
        return False
    try:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # PWM outputs
        GPIO.setup(PIN_L, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_R, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PIN_B, GPIO.OUT, initial=GPIO.LOW)

        _GPIO_PWM_L = GPIO.PWM(PIN_L, 50)
        _GPIO_PWM_R = GPIO.PWM(PIN_R, 50)
        _GPIO_PWM_B = GPIO.PWM(PIN_B, 50)
        _GPIO_PWM_L.start(_pwm_us_to_duty(PWM_NEUTRAL))
        _GPIO_PWM_R.start(_pwm_us_to_duty(PWM_NEUTRAL))
        _GPIO_PWM_B.start(_pwm_us_to_duty(PWM_NEUTRAL))

        # Ultrasonic pins
        for s in US_SENSORS:
            GPIO.setup(s["trig"], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(s["echo"], GPIO.IN)
            GPIO.output(s["trig"], GPIO.LOW)

        _GPIO_READY = True
        print(
            f"[GPIO] ready PWM L={PIN_L} R={PIN_R} B={PIN_B}; "
            f"US={[(s['name'], s['trig'], s['echo']) for s in US_SENSORS]}",
            flush=True,
        )
        return True
    except Exception as e:
        if (time.time() - _LAST_FAIL_TS) > 2.0:
            print(f"[GPIO] init failed: {e}", flush=True)
            _LAST_FAIL_TS = time.time()
        _gpio_soft_close()
        return False


def poll_arduino_startstop(on_toggle_cb):
    """
    Legacy API kept for compatibility.
    UNO is removed; start/stop from serial is disabled.
    """
    return


_prev_send_ts = 0.0
_KEEPALIVE_SEC = 0.10  # 100 мс (можно 0.05)

def _send_m(l_us: int, r_us: int, b_us: int) -> None:
    """
    Send command directly to GPIO PWM channels.
    """
    global _prev_lrb, _prev_send_ts, _LAST_SENT, _LAST_FAIL_TS

    l = _clamp_servo_us(l_us)
    r = _clamp_servo_us(r_us)
    b = _clamp_servo_us(b_us)

    triple = (l, r, b)
    now = time.time()

    # Шлём если изменилось ИЛИ прошло время keepalive
    if triple == _prev_lrb and (now - _prev_send_ts) < _KEEPALIVE_SEC:
        return

    if not _ensure_gpio():
        return

    try:
        _GPIO_PWM_L.ChangeDutyCycle(_pwm_us_to_duty(l))
        _GPIO_PWM_R.ChangeDutyCycle(_pwm_us_to_duty(r))
        _GPIO_PWM_B.ChangeDutyCycle(_pwm_us_to_duty(b))

        _LAST_SENT = triple
        _prev_lrb = triple
        _prev_send_ts = now

        if (now - getattr(_send_m, "_last_dbg_ts", 0.0)) > 0.2:
            _send_m._last_dbg_ts = now

    except Exception as e:
        print(f"[GPIO] PWM write error: {e}", flush=True)
        _LAST_FAIL_TS = time.time()
        _gpio_soft_close()


def _measure_distance_cm(trig_pin: int, echo_pin: int, timeout_s: float = 0.03):
    if GPIO is None:
        return None
    try:
        GPIO.output(trig_pin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig_pin, GPIO.LOW)

        t0 = time.monotonic()
        while GPIO.input(echo_pin) == 0:
            if (time.monotonic() - t0) > timeout_s:
                return None
        pulse_start = time.monotonic()

        while GPIO.input(echo_pin) == 1:
            if (time.monotonic() - pulse_start) > timeout_s:
                return None
        pulse_end = time.monotonic()

        dist_cm = (pulse_end - pulse_start) * 17150.0
        if dist_cm < 2.0 or dist_cm > 450.0:
            return None
        return float(dist_cm)
    except Exception:
        return None


def _read_ultra_all_cached(sample_period_s: float = 0.12):
    global _US_LAST_TS, _US_CACHE
    now = time.monotonic()
    if (now - _US_LAST_TS) < float(sample_period_s):
        return dict(_US_CACHE)
    if not _ensure_gpio():
        return dict(_US_CACHE)
    out = {}
    for s in US_SENSORS:
        out[s["name"]] = _measure_distance_cm(s["trig"], s["echo"])
    _US_CACHE = out
    _US_LAST_TS = now
    return dict(_US_CACHE)


def _parking_safety_block(state, l_us: int, r_us: int):
    """
    Returns (blocked, reason) based on ultrasonic sensors.
    """
    global _PARK_BLOCK_ACTIVE, _PARK_BLOCK_REASON
    l = int(l_us)
    r = int(r_us)
    moving = abs(l - PWM_NEUTRAL) > 20 or abs(r - PWM_NEUTRAL) > 20
    forward = (l > PWM_NEUTRAL + 10) and (r > PWM_NEUTRAL + 10)

    dist = _read_ultra_all_cached()
    try:
        state.park_front_left_cm = dist.get("front_left")
        state.park_front_right_cm = dist.get("front_right")
        state.park_side_left_cm = dist.get("side_left")
        state.park_side_right_cm = dist.get("side_right")
    except Exception:
        pass

    front_thr = float(getattr(state, "park_front_stop_cm", 45.0) or 45.0)
    side_thr = float(getattr(state, "park_side_stop_cm", 30.0) or 30.0)
    blocked = False
    reason = ""

    fl = dist.get("front_left")
    fr = dist.get("front_right")
    sl = dist.get("side_left")
    sr = dist.get("side_right")

    if forward:
        if (fl is not None and fl < front_thr) or (fr is not None and fr < front_thr):
            blocked = True
            reason = f"front obstacle fl={fl} fr={fr} cm (<{front_thr:.0f})"

    if (not blocked) and moving:
        if (sl is not None and sl < side_thr) or (sr is not None and sr < side_thr):
            blocked = True
            reason = f"side obstacle sl={sl} sr={sr} cm (<{side_thr:.0f})"

    if blocked:
        if (not _PARK_BLOCK_ACTIVE) or (reason != _PARK_BLOCK_REASON):
            print(f"[PARKTRONIC] STOP {reason}", flush=True)
        _PARK_BLOCK_ACTIVE = True
        _PARK_BLOCK_REASON = reason
    else:
        if _PARK_BLOCK_ACTIVE:
            print("[PARKTRONIC] clear", flush=True)
        _PARK_BLOCK_ACTIVE = False
        _PARK_BLOCK_REASON = ""

    return blocked, reason
# ================== МОТОРЫ / ИНСТРУМЕНТ ==================
def motors_set(state, l_pwm, r_pwm, b_pwm=None):
    """
    Универсальная функция:
      - нормализует значения до диапазона сервосигнала,
      - шлёт команду на Arduino,
      - обновляет state, чтобы HUD и логика знали реальные PWM.

    trim применяется:
      - ВПЕРЁД  (l>1500 and r>1500):   l += trim, r -= trim
      - НАЗАД   (l<1500 and r<1500):   l -= trim, r += trim
      - ИНАЧЕ (повороты/стоп):         без trim
    """
    l = _clamp_servo_us(l_pwm)
    r = _clamp_servo_us(r_pwm)

    # ✅ если b_pwm=None — НЕ СБРАСЫВАЕМ, а сохраняем текущий B
    if b_pwm is None:
        b_cur = getattr(state, "b_pwm", None)
        if b_cur is None:
            b_cur = getattr(state, "manual_b_pwm", PWM_NEUTRAL)
        b = _clamp_servo_us(b_cur)
    else:
        b = _clamp_servo_us(b_pwm)

    try:
        trim = int(getattr(state, "pwm_bias", 0) or 0)
    except Exception:
        trim = 0

    # --------------------------------------------------
    # APPLY TRIM DEPENDING ON DIRECTION
    # --------------------------------------------------
    if l > PWM_NEUTRAL and r > PWM_NEUTRAL:
        # движение вперёд
        l = l + trim
        r = r - trim

    elif l < PWM_NEUTRAL and r < PWM_NEUTRAL:
        # движение назад — ИНВЕРСИЯ trim
        l = l - trim
        r = r + trim

    # else:
    #   поворот на месте или стоп — trim НЕ применяем
    # --------------------------------------------------

    # финальный clamp на всякий случай
    l = _clamp_servo_us(l)
    r = _clamp_servo_us(r)

    # parktronic safety stop (direct GPIO mode)
    blocked, reason = _parking_safety_block(state, l, r)
    if blocked:
        l = PWM_NEUTRAL
        r = PWM_NEUTRAL
        try:
            state.safety_stop_ultra = True
            state.safety_stop_reason = str(reason)
            front = bool(getattr(state, "safety_stop_front", False))
            rear = bool(getattr(state, "safety_stop_rear", False))
            state.safety_stop = bool(front or rear or state.safety_stop_ultra)
        except Exception:
            pass
    else:
        try:
            state.safety_stop_ultra = False
            front = bool(getattr(state, "safety_stop_front", False))
            rear = bool(getattr(state, "safety_stop_rear", False))
            state.safety_stop = bool(front or rear)
        except Exception:
            pass

    #print(f"[SENT] = L:{l} R:{r} B:{b}", flush=True)

    _send_m(l, r, b)

    # сохраняем в state для HUD/логики
    state.l_pwm = l
    state.r_pwm = r
    state.b_pwm = b

    state.manual_l_pwm = l
    state.manual_r_pwm = r
    state.manual_b_pwm = b


def motors_stop(state) -> None:
    motors_set(state, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL)
    state.speed_mps = 0.0


def tool_pulse(
    ui: QtWidgets.QMainWindow,
    state,
    level_us: int = PWM_MAX,
    ms: int = 200
) -> None:
    """
    Краткий импульс по вспомогательному каналу B:
      - на ms миллисекунд ставим B = level_us
      - потом возвращаем прежний B.
    """
    # fallback: у тебя везде l_pwm/r_pwm/b_pwm
    base_L = int(getattr(state, "l_pwm", getattr(state, "motor_L", PWM_NEUTRAL)) or PWM_NEUTRAL)
    base_R = int(getattr(state, "r_pwm", getattr(state, "motor_R", PWM_NEUTRAL)) or PWM_NEUTRAL)
    base_B = int(getattr(state, "b_pwm", getattr(state, "motor_B", PWM_NEUTRAL)) or PWM_NEUTRAL)

    motors_set(state, base_L, base_R, level_us)

    def _off():
        motors_set(state, base_L, base_R, base_B)

    QtCore.QTimer.singleShot(max(1, int(ms)), _off)


# ================== СКОРОСТЬ ДЛЯ HUD ==================
def get_speed(state) -> float:
    """
    Оцениваем линейную скорость из текущих PWM моторов по ТАБЛИЦЕ.
    """
    l = float(getattr(state, "manual_l_pwm", PWM_NEUTRAL) or PWM_NEUTRAL)
    r = float(getattr(state, "manual_r_pwm", PWM_NEUTRAL) or PWM_NEUTRAL)
    avg = (l + r) / 2.0

    if avg > PWM_NEUTRAL + 1e-6:
        sign = 1.0
    elif avg < PWM_NEUTRAL - 1e-6:
        sign = -1.0
    else:
        sign = 0.0

    step = 50
    pwm_key = int(round(avg / step) * step)
    base_speed = SPEED_TABLE.get(pwm_key, SPEED_DEFAULT_MPS if sign != 0.0 else 0.0)

    v_ms = sign * base_speed
    state.speed_mps = v_ms
    return v_ms


def set_guard_state(state, value: str):
    state.guard_state = str(value)


def set_safety_stop(ui: QtWidgets.QMainWindow, enabled: bool, state=None):
    w = ui.findChild(QtWidgets.QLabel, "lblSafetyStop")
    if w:
        if enabled:
            w.setStyleSheet("color:#ffffff; background:#d64545; padding:2px 6px; border-radius:4px;")
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


def apply_start_defaults(state) -> None:
    start_us = int(getattr(state, "drive_pwm_us", 1500) or 1500)
    motors_set(state, start_us, start_us, state.b_pwm)


def close_link() -> None:
    """Compatibility API for app shutdown."""
    _gpio_soft_close()


# ================== HUD ==================
from routing import route_caption_text  # используем версию из routing.py


def update_drive_panel(ui: QtWidgets.QMainWindow, state) -> None:
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

