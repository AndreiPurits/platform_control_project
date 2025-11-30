# -*- coding: utf-8 -*-
"""
lidar_matcher.py

Модуль для:
  - построения базы эталонных лидар-сканов вдоль маршрута (из JSON файлов),
  - онлайн-сравнения текущего скана с базой,
  - аккуратного «прищёлкивания» (snap) позиции робота к ближайшей
    эталонной точке при хорошем совпадении.

Формат JSON в базе (по одному файлу на точку маршрута):
  {
    "px":  [x_px, y_px],      # координаты на карте (в пикселях)
    "pts": [[x_m, y_m], ...]  # точки лидара в МЕТРАХ в системе робота
  }

Рекомендуемая директория:
  ~/roadseg_work/datasets/lidar/Poly_asf
"""

import os
import glob
import json
import math
from typing import List, Tuple, Optional

import numpy as np

Point = Tuple[float, float]


# ---------------------------------------------------------------------------
#   ХЕЛПЕРЫ: ПРЕОБРАЗОВАНИЕ СКАНА В ПОЛЯРНЫЙ ПРИЗНАК
# ---------------------------------------------------------------------------

def _scan_to_polar(
    pts: List[Point],
    ang_span_deg: float = 90.0,
    ang_bins: int = 90,
    r_max: float = 8.0,
    r_bins: int = 40,
) -> np.ndarray:
    """
    Преобразует набор точек (x,y) в метрах в «полярный» вектор признаков.

    pts          : список точек [(x_m, y_m), ...] в СК робота
    ang_span_deg : общий угол обзора (по умолчанию 90° → ±45°),
                   здесь мы специально используем ±90°, т.е. спереди + по бокам.
    ang_bins     : число бинов по углу
    r_max        : макс. расстояние (м)
    r_bins       : число бинов по радиусу

    Возвращает: np.ndarray формы (ang_bins * r_bins,) float32.
    """
    if not pts:
        return np.zeros((ang_bins * r_bins,), np.float32)

    half = math.radians(ang_span_deg)  # половина угла (в радианах)
    feats = np.zeros((ang_bins, r_bins), np.float32)

    for x, y in pts:
        # угол: 0 — вперёд, положительный — влево, отрицательный — вправо
        ang = math.atan2(y, x)
        if abs(ang) > half:
            # слишком сбоку / сзади — игнорируем
            continue

        r = math.hypot(x, y)
        if r > r_max or r <= 1e-3:
            continue

        # индексы по углу и радиусу
        a = int((ang + half) / (2.0 * half) * (ang_bins - 1))
        b = int(r / r_max * (r_bins - 1))

        # ближе — сильнее вес
        val = 1.0 - r / r_max
        if val > feats[a, b]:
            feats[a, b] = val

    return feats.flatten()


def _cos_sim(a: np.ndarray, b: np.ndarray) -> float:
    """
    Косинусное сходство двух векторов.
    """
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na < 1e-6 or nb < 1e-6:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


# ---------------------------------------------------------------------------
#   LIDAR MATCHER
# ---------------------------------------------------------------------------

class LidarMatcher:
    """
    LidarMatcher:
      - build_from_dir(...) — собирает базу из JSON-файлов.
      - match(cur_pts, ...) — ищет лучшее совпадение с текущим сканом.

    База:
      self.db = list[(x_px, y_px, feat_vector, n_pts_db)]
    """

    def __init__(self):
        self.db: List[Tuple[int, int, np.ndarray, int]] = []

    # --- СБОРКА БАЗЫ ИЗ ДИРЕКТОРИИ ----------------------------------------

    def build_from_dir(self, lidar_dir: str) -> None:
        """
        Читает все *.json из lidar_dir и строит базу лидар-паттернов.

        Ожидаемый формат JSON:
          {
            "px":  [x_px, y_px],
            "pts": [[x_m, y_m], ...]
          }
        """
        self.db.clear()

        files = glob.glob(os.path.join(lidar_dir, "*.json"))
        print(f"[LIDMATCH] scanning dir={lidar_dir}, files={len(files)}", flush=True)

        for p in files:
            try:
                with open(p, "r", encoding="utf-8") as f:
                    z = json.load(f)

                px = z.get("px", None)
                pts = z.get("pts", None)

                if (
                    not isinstance(px, (list, tuple))
                    or len(px) != 2
                    or not pts
                ):
                    continue
                x_px, y_px = px
                x_px = int(x_px)
                y_px = int(y_px)

                # pts — это список [x_m, y_m]
                pts_list: List[Point] = []
                for t in pts:
                    if not isinstance(t, (list, tuple)) or len(t) != 2:
                        continue
                    xm, ym = float(t[0]), float(t[1])
                    pts_list.append((xm, ym))

                if not pts_list:
                    continue

                n_pts = len(pts_list)
                feat = _scan_to_polar(pts_list)  # уже с углом ±90°

                self.db.append((x_px, y_px, feat, n_pts))

            except Exception as e:
                print(f"[LIDMATCH] load error {p}: {e}", flush=True)

        print(f"[LIDMATCH] loaded entries: {len(self.db)}", flush=True)

    # --- ПОИСК ЛУЧШЕГО СОВПАДЕНИЯ -----------------------------------------

    def match(
        self,
        cur_pts: List[Point],
        min_points: int = 30,
    ) -> Tuple[Optional[int], Optional[int], float]:
        """
        Сравнивает текущий скан cur_pts с базой.

        cur_pts   : [(x_m, y_m), ...] в МЕТРАХ в СК робота
        min_points: минимальное число точек, чтобы матч считать осмысленным

        Возвращает:
          (best_x_px, best_y_px, score)
          либо (None, None, 0.0), если база пуста / мало точек / пустой признак.
        """
        if not self.db:
            return None, None, 0.0

        if not cur_pts or len(cur_pts) < min_points:
            print(f"[LIDMATCH] skip: pts={len(cur_pts)} < {min_points}", flush=True)
            return None, None, 0.0

        q = _scan_to_polar(cur_pts)  # те же параметры, что и в базе

        if float(np.sum(q)) <= 1e-6:
            print("[LIDMATCH] skip: empty feature vector", flush=True)
            return None, None, 0.0

        best_x: Optional[int] = None
        best_y: Optional[int] = None
        best_score: float = 0.0

        for (x_px, y_px, feat, n_pts_db) in self.db:
            s = _cos_sim(q, feat)
            if s > best_score:
                best_score = s
                best_x = x_px
                best_y = y_px

        return best_x, best_y, best_score


# ---------------------------------------------------------------------------
#   SNAP ПОЗИЦИИ РОБОТА ПО ЛИДАРУ
# ---------------------------------------------------------------------------

def _cumlen_m(poly_m: List[Point]) -> List[float]:
    """
    Кумулятивные длины вдоль ломаной poly_m (в МЕТРАХ).
    """
    acc = [0.0]
    total = 0.0
    for a, b in zip(poly_m, poly_m[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        total += d
        acc.append(total)
    return acc


def snap_robot_by_lidar(
    state,
    matcher: LidarMatcher,
    cur_pts: List[Point],
    min_points: int = 30,
    min_score: float = 0.95,
    max_route_diff_m: float = 5.0,
) -> None:
    """
    Пытается «прищёлкнуть» позицию робота по текущему лидар-скану.

    Условия:
      - len(cur_pts) >= min_points
      - score >= min_score
      - новый route_done_m не отличается от текущего больше, чем на max_route_diff_m

    Требуется в state:
      - route_pts_px : [(x_px, y_px), ...]   — ломаная маршрута в px
      - route_pts_m  : [(X_m, Y_m), ...]     — та же ломаная в МЕТРАХ
      - route_done_m : float                 — текущий прогресс по маршруту
      - robot_px     : (x_px, y_px)          — будет обновлён при успешном snap
    """
    poly_px: List[Point] = getattr(state, "route_pts_px", None) or []
    poly_m:  List[Point] = getattr(state, "route_pts_m",  None) or []

    if not poly_px or not poly_m or len(poly_px) != len(poly_m):
        return

    best_x, best_y, score = matcher.match(cur_pts, min_points=min_points)

    if best_x is None or best_y is None:
        return

    if score < min_score:
        if score > 0.5:
            print(
                f"[LIDMATCH] match score={score:.3f} < {min_score:.2f}, no snap",
                flush=True,
            )
        return
    n_pts_cur = len(cur_pts)
    print(
        f"[LIDMATCH] good match: score={score:.3f}, pts={n_pts_cur}, "
        f"px=({best_x},{best_y})",
        flush=True,
    )

    # --- найдём ближайшую точку маршрута к (best_x,best_y) ---
    bx, by = float(best_x), float(best_y)
    best_i = 0
    best_d2 = float("inf")

    for i, (x, y) in enumerate(poly_px):
        d2 = (x - bx) ** 2 + (y - by) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i

    # пересчитываем прогресс вдоль маршрута
    cum_m = _cumlen_m(poly_m)
    if not cum_m:
        return

    idx = max(0, min(len(cum_m) - 1, best_i))
    new_done = float(cum_m[idx])
    cur_done = float(getattr(state, "route_done_m", 0.0) or 0.0)

    delta = new_done - cur_done
    if abs(delta) > max_route_diff_m:
        # слишком большой скачок — игнорируем
        print(
            f"[LIDMATCH] snap rejected: Δs={delta:+.2f} м > {max_route_diff_m:.1f} м",
            flush=True,
        )
        return

    # всё ок → обновляем позицию и прогресс
    state.robot_px = (bx, by)
    state.route_done_m = new_done

    print(
        f"[LIDMATCH] SNAP: robot_px→({bx:.1f},{by:.1f}), "
        f"route_done_m {cur_done:.2f}→{new_done:.2f}",
        flush=True,
    )