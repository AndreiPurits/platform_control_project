#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Генерация шахматки для калибровки камеры.

Важно:
- Для cv2.findChessboardCorners задаются *внутренние углы* (inner corners).
- Печатаем на A4 (по умолчанию 210x297 мм).
- square_mm — размер клетки в мм (ЭТО то, что потом укажешь в калибровке).

Выход:
- chessboard_A4.png (300 DPI эквивалент в пикселях)
"""

import math
import cv2
import numpy as np

# ---------------- ПАРАМЕТРЫ ПЕЧАТИ ----------------
PAGE_W_MM = 210
PAGE_H_MM = 297

DPI = 300  # чем выше, тем чётче печать
MARGIN_MM = 10  # поля на листе

# ---------------- ПАРАМЕТРЫ ШАХМАТКИ ----------------
# INNER corners (для поиска углов)
INNER_CORNERS_X = 9
INNER_CORNERS_Y = 6

# Размер клетки (мм) — выбери удобно для печати и точности:
# 25 мм / 30 мм / 35 мм обычно хорошо.
SQUARE_MM = 25

OUT_PNG = "chessboard_A4.png"


def mm_to_px(mm: float, dpi: int) -> int:
    return int(round(mm * dpi / 25.4))


def main():
    page_w_px = mm_to_px(PAGE_W_MM, DPI)
    page_h_px = mm_to_px(PAGE_H_MM, DPI)
    margin_px = mm_to_px(MARGIN_MM, DPI)

    # шахматка в "клетках" = inner + 1
    squares_x = INNER_CORNERS_X + 1
    squares_y = INNER_CORNERS_Y + 1

    sq_px = mm_to_px(SQUARE_MM, DPI)

    board_w = squares_x * sq_px
    board_h = squares_y * sq_px

    # проверим, что влезает в A4
    max_w = page_w_px - 2 * margin_px
    max_h = page_h_px - 2 * margin_px
    if board_w > max_w or board_h > max_h:
        # если не влезает — уменьшаем клетку автоматически
        scale = min(max_w / board_w, max_h / board_h)
        sq_px = int(math.floor(sq_px * scale))
        board_w = squares_x * sq_px
        board_h = squares_y * sq_px
        print(f"[WARN] Board too big, auto-scale square to ~{sq_px}px")

    # белый лист
    img = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    # позиция по центру
    x0 = (page_w_px - board_w) // 2
    y0 = (page_h_px - board_h) // 2

    # рисуем клетки
    for y in range(squares_y):
        for x in range(squares_x):
            if (x + y) % 2 == 0:
                x1 = x0 + x * sq_px
                y1 = y0 + y * sq_px
                img[y1:y1 + sq_px, x1:x1 + sq_px] = 0

    cv2.imwrite(OUT_PNG, img)
    print(f"[OK] Saved: {OUT_PNG}")
    print("Печать: 100% scale (без 'fit to page').")
    print(f"INNER corners = ({INNER_CORNERS_X},{INNER_CORNERS_Y})")
    print(f"SQUARE size = {SQUARE_MM} mm (используй это в калибровке!)")


if __name__ == "__main__":
    main()