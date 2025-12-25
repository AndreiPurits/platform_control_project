# -*- coding: utf-8 -*-
"""
hw_ports.py — централизованная конфигурация последовательных портов.

Задачи:
  - Разнести Arduino / лидары по разным портам
  - Позволить переопределять порты через env-переменные
  - Избежать ситуации, когда robot_cmd лезет в порт лидара
"""

import os
import glob

def _resolve_symlink(path: str) -> str:
    try:
        return os.path.realpath(path)
    except Exception:
        return path


def get_arduino_port():
    """
    Возвращаем ЖЁСТКО фиксированный порт для Arduino — /dev/ttyUSB0.
    """
    cand = "/dev/ttyUSB0"
    if os.path.exists(cand):
        return cand
    return None

def get_lidar_front_port() -> str | None:
    """
    Порт фронтального лидара (например, RPLidar на CP210x).
    Его важно отличать от Arduino.
    """
    env_port = os.environ.get("ROVER_LIDAR_FRONT")
    if env_port:
        return env_port

    # На практике обычно это первый CP210x-устрой
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    for p in by_id:
        name = os.path.basename(p).lower()
        if "silicon_labs" in name or "cp210" in name:
            # первый CP210x считаем фронтальным лидаром
            return _resolve_symlink(p)

    # fallback — если нет by-id, можно взять первый ttyUSB*
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if usb:
        return usb[0]

    return None


def get_lidar_rear_port() -> str | None:
    """
    Порт второго лидара (например, заднего).
    По умолчанию предполагаем, что это второй CP210x / ttyUSB.
    """
    env_port = os.environ.get("ROVER_LIDAR_REAR")
    if env_port:
        return env_port

    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    cp210 = [p for p in by_id if "silicon_labs" in os.path.basename(p).lower()
                                   or "cp210" in os.path.basename(p).lower()]
    if len(cp210) >= 2:
        return _resolve_symlink(cp210[1])

    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if len(usb) >= 2:
        return usb[1]

    return None