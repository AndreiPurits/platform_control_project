# -*- coding: utf-8 -*-  # lidar_matcher.py
import os
import glob
import json
import math
from typing import List, Tuple, Optional

import numpy as np

Point = Tuple[float, float]

# ---------- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ----------

def _scan_to_polar(
    pts: List[Point],
    ang_span_deg: float = 90.0,
    ang_bins: int = 60,
    r_max: float = 8.0,
    r_bins: int = 40,
) -> np.ndarray:
    """
    pts: список (x_m, y_m) в МЕТРАХ, в системе робота.
         x — вперёд, y — влево (как обычно для лидара).

    Возвращает вектор признаков формы:
        (ang_bins * r_bins,)  float32

    Учитываем ТОЛЬКО сектор ±ang_span_deg/2 (по умолчанию ±90°).
    """
    if not pts:
        return np.zeros((ang_bins * r_bins,), np.float32)

    half = math.radians(ang_span_deg)
    feats = np.zeros((ang_bins, r_bins), np.float32)

    for x, y in pts:
        ang = math.atan2(y, x)  # 0 = вперёд, +влево
        if abs(ang) > half:
            continue
        r = math.hypot(x, y)
        if r > r_max or r <= 1e-3:
            continue

        # бин по углу [−half, +half] -> [0 .. ang_bins-1]
        a = int((ang + half) / (2 * half) * (ang_bins - 1))
        # бин по радиусу [0, r_max] -> [0 .. r_bins-1]
        b = int(r / r_max * (r_bins - 1))

        # ближе к роботу — более сильный вклад
        feats[a, b] = max(feats[a, b], 1.0 - r / r_max)

    return feats.flatten().astype(np.float32)

def _cos_sim(a: np.ndarray, b: np.ndarray) -> float:
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return float(np.dot(a, b) / (na * nb))

# ---------- КЛАСС МЭТЧЕРА ----------

class LidarMatcher:
    """
    Простая БД "слепков" лидара вдоль маршрута.

    Каждый элемент:
        (x_px, y_px, feat_vector)
    где feat_vector — результат _scan_to_polar([...]).
    """

    def __init__(self):
        self.db: List[Tuple[int, int, np.ndarray]] = []

    # ---------------- БИЛДЕР ----------------
    def build_from_dir(self, lidar_dir: str):
        """
        Загружает *.json из директории lidar_dir.

        Формат json:
            {
              "px": [x_px, y_px],
              "pts": [[x_m, y_m], ...]   # в МЕТРАХ
            }
        """
        self.db.clear()
        if not os.path.isdir(lidar_dir):
            print(f"[LIDMATCH] dir not found: {lidar_dir}", flush=True)
            return

        files = sorted(glob.glob(os.path.join(lidar_dir, "*.json")))
        loaded = 0

        for p in files:
            try:
                with open(p, "r", encoding="utf-8") as f:
                    z = json.load(f)
                px = z.get("px", None)
                pts = z.get("pts", None)
                if (
                    not isinstance(px, (list, tuple))
                    or len(px) != 2
                    or not isinstance(pts, list)
                    or not pts
                ):
                    continue

                x_px, y_px = float(px[0]), float(px[1])
                # pts — список [x_m, y_m]
                pts_m: List[Point] = []
                for q in pts:
                    if not isinstance(q, (list, tuple)) or len(q) != 2:
                        continue
                    pts_m.append((float(q[0]), float(q[1])))

                if not pts_m:
                    continue

                feat = _scan_to_polar(pts_m)  # ±90°
                self.db.append((int(x_px), int(y_px), feat))
                loaded += 1
            except Exception as e:
                print(f"[LIDMATCH] error loading {p}: {e}", flush=True)

        print(f"[LIDMATCH] loaded entries: {loaded} from {lidar_dir}", flush=True)

    # ---------------- MATCH ----------------
    def match(
        self,
        cur_pts_m: List[Point],
        approx_px: Optional[Point] = None,
        window_px: Optional[float] = None,
    ) -> Tuple[Optional[float], Optional[float], float]:
        """
        Сравнивает текущий скан с БД.

        cur_pts_m: текущий скан [(x_m, y_m), ...] в системе робота.
        approx_px: примерное положение робота (x_px, y_px— чтобы ограничить поиск.
        window_px: если задано, берём из БД только точки с dist(approx_px, db_px) <= window_px.

        Возвращает:
            (best_x_px, best_y_px, best_score)
        либо (None, None, 0.0), если ничего толкового нет.
        """
        if not self.db:
            return None, None, 0.0

        q = _scan_to_polar(cur_pts_m)  # дескриптор текущего скана
        if float(np.sum(q)) <= 1e-6:
            return None, None, 0.0

        best_x, best_y, best_score = None, None, 0.0

        for (x_px, y_px, feat) in self.db:
            # если задано approx_px и окно — сузим поисковое пространство
            if approx_px is not None and window_px is not None:
                dx = x_px - approx_px[0]
                dy = y_px - approx_px[1]
                if dx * dx + dy * dy > window_px * window_px:
                    continue

            s = _cos_sim(q, feat)
            if s > best_score:
                best_x, best_y, best_score = x_px, y_px, s

        return best_x, best_y, best_score
        

# ---------- SNAP ПО ОТВЕТУ МЭТЧЕРА ----------

def snap_robot_by_lidar(
    state,
    matcher: LidarMatcher,
    cur_pts_m: List[Point],
    min_points: int = 30,
    min_score: float = 0.95,
    max_route_diff_m: float = 5.0,
) -> bool:
    """
    Пытается "прищёлкнуть" позицию робота по лидарам.

    Использует:
    - state.robot_px         — текущая оценка позиции (px)
    - state.route_pts_px     — маршрут (для масштаба)
    - state.ppm / state.PPM  — пикселей на метр

    Условия:
    - точек в текущем скане >= min_points;
    - косинусное сходство >= min_score;
    - новое положение не дальше max_route_diff_m от текущего (по карте).

    При успехе:
    - обновляет state.robot_px;
    - пишет лог;
    - возвращает True.

    При неуспехе — False.
    """
    if matcher is None or not isinstance(matcher, LidarMatcher) or not matcher.db:
        # нет базы — нечего делать
        return False

    if not cur_pts_m or len(cur_pts_m) < min_points:
        # слишком мало точек
        return False

    robot_px = getattr(state, "robot_px", None)
    if robot_px is None:
        # нет текущей оценки — пока не снапаем
        return False

    # пиксели на метр
    ppm = float(getattr(state, "ppm", getattr(state, "PPM", 80.0)) or 80.0)
    max_diff_px = max_route_diff_m * ppm

    # матчим с ограничением по окну вокруг текущей позиции
    approx_px = (float(robot_px[0]), float(robot_px[1]))
    best_x, best_y, score = matcher.match(
        cur_pts_m,
        approx_px=approx_px,
        window_px=max_diff_px,
    )

    if best_x is None or best_y is None:
        return False

    if score < min_score:
        # мало похоже — игнорируем
        print(
            f"[LIDMATCH] low score: {score:.3f} < {min_score:.3f}, snap skipped",
            flush=True,
        )
        return False

    # Проверка по расстоянию (дополнительно к window_px, на всякий случай)
    dx = best_x - approx_px[0]
    dy = best_y - approx_px[1]
    dist_px = math.hypot(dx, dy)
    if dist_px > max_diff_px:
        print(
            f"[LIDMATCH] candidate too far: {dist_px/ppm:.2f} m > {max_route_diff_m:.2f} m, snap skipped",
            flush=True,
        )
        return False

    # УСПЕХ: переносим позицию робота
    old_x, old_y = approx_px
    state.robot_px = (float(best_x), float(best_y))

    print(
        f"[LIDMATCH] SNAP OK: score={score:.3f}, pts={len(cur_pts_m)} "
        f"({old_x:.1f},{old_y:.1f}) -> ({best_x:.1f},{best_y:.1f})",
        flush=True,
    )

    return True