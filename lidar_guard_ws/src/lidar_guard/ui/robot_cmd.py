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
    1500: 0.01,
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
    #print(f"[HW] Arduino port resolved: {port}", flush=True)
    return "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"


def _serial_soft_close() -> None:
    """Аккуратно закрыть порт и сбросить глобальные переменные."""
    global _SER, _SER_PORT
    try:
        if _SER is not None:
            try:
                _SER.close()
            except Exception:
                pass
    finally:
        _SER = None
        _SER_PORT = None


def _ensure_serial() -> bool:
    """
    Открываем порт, если ещё не открыт.
    Поведение:
      - если порт отвалился -> закрываем и пробуем переподключиться часто
      - троттлинг ошибок, чтобы не спамить
    """
    global _SER, _SER_PORT, _LAST_FAIL_TS

    if serial is None:
        if time.time() - _LAST_FAIL_TS > 3.0:
            #print("[ARDUINO] pyserial не установлен — команды не отправляются", flush=True)
            _LAST_FAIL_TS = time.time()
        return False

    if _SER and getattr(_SER, "is_open", False):
        return True

    # переподключаемся часто, но не в tight-loop
    if time.time() - _LAST_FAIL_TS < 0.25:
        return False

    port = _pick_port()
    if not port:
        # порт не найден (отвалился / не смонтировался)
        if time.time() - _LAST_FAIL_TS > 2.0:
            #print("[ARDUINO] порт не найден — жду переподключения", flush=True)
            _LAST_FAIL_TS = time.time()
        _serial_soft_close()
        return False

    try:
        _SER = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=0.01,
            write_timeout=0.05,
        )
        try:
            _SER.reset_input_buffer()
            _SER.reset_output_buffer()
        except Exception:
            pass

        _SER_PORT = port
        print(f"[ARDUINO] connected: {port}", flush=True)
        return True

    except Exception as e:
        if time.time() - _LAST_FAIL_TS > 2.0:
            #print(f"[ARDUINO] open fail {port}: {e}", flush=True)
            _LAST_FAIL_TS = time.time()
        _serial_soft_close()
        return False


def _send_m(l_us: int, r_us: int, b_us: int) -> None:
    """
    Отправка команды на Arduino в СЕРВО-ФОРМАТЕ:
      M L=<1000..2000> R=<1000..2000> B=<1000..2000>\n
    При ошибке записи:
      - закрываем порт
      - в следующий вызов будет попытка переподключения
    """
    global _prev_lrb, _LAST_SENT, _LAST_FAIL_TS

    l = _clamp_servo_us(l_us)
    r = _clamp_servo_us(r_us)
    b = _clamp_servo_us(b_us)

    triple = (l, r, b)
    if triple == _prev_lrb:
        # команда не изменилась — но всё равно полезно иногда слать "держание"
        # здесь НЕ шлём повторно, чтобы не грузить систему
        return
    _prev_lrb = triple

    if not _ensure_serial():
        return

    msg = f"M L={l} R={r} B={b}\n"
    try:
        _SER.write(msg.encode("ascii", errors="ignore"))
        _LAST_SENT = (l, r, b)
    except Exception as e:
        # ВАЖНО: при write error закрываем порт, дальше будет реконнект
        #print(f"[ARDUINO] write error: {e} (port lost, will reconnect)", flush=True)
        _LAST_FAIL_TS = time.time()
        _serial_soft_close()


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
        trim = int(getattr(state, "pwm_trim_lr", 0) or 0)
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
    start_us = int(getattr(state, "drive_pwm_us", 1700) or 1700)
    motors_set(state, start_us, start_us, PWM_NEUTRAL)


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

