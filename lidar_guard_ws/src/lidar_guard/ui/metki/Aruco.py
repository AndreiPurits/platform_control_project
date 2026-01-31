import io
import numpy as np
import cv2

from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

# =========================
# CONFIG
# =========================
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

MARKER_MM = 180.0
PNG_PX = 1600  # качество (можно 1200-2000)
TEXT_GAP_MM = 8

# ID -> (label, action text)
MARKERS = {
    10: ("ПРЯМО", "прямо — столб справа"),
    15: ("ПРЯМО", "прямо — столб слева"),
    20: ("ПОВОРОТ НАПРАВО", "поворот направо"),
    30: ("ПОВОРОТ НАЛЕВО", "поворот налево"),
}

OUT_PDF = "A4_ArUco_180mm_ID10_15_20_30.pdf"

# =========================
# HELPERS
# =========================
def aruco_png_bytes(tag_id: int) -> bytes:
    # Самый совместимый способ: DICT.drawMarker()
    img = np.zeros((PNG_PX, PNG_PX), dtype=np.uint8)
    DICT.drawMarker(tag_id, PNG_PX, img, 1)

    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise RuntimeError("PNG encode failed")
    return buf.tobytes()

def draw_marker_page(c, tag_id: int):
    w_pt, h_pt = A4
    label, text = MARKERS[tag_id]

    img = ImageReader(io.BytesIO(aruco_png_bytes(tag_id)))
    size_pt = MARKER_MM * mm

    cx = w_pt / 2
    cy = (h_pt / 2) + 18 * mm  # сдвиг вверх, чтобы подпись влезла снизу

    x = cx - size_pt / 2
    y = cy - size_pt / 2

    c.drawImage(img, x, y, width=size_pt, height=size_pt, mask="auto")

    # подписи снизу
    c.setFont("DejaVuSans-Bold", 16)
    c.drawCentredString(cx, y - TEXT_GAP_MM * mm, label)

    c.setFont("DejaVuSans", 12)
    c.drawCentredString(cx, y - (TEXT_GAP_MM + 8) * mm, f"ID {tag_id} — {text}")

def make_pdf(filename: str, ids):
    c = canvas.Canvas(filename, pagesize=A4)
    for tag_id in ids:
        draw_marker_page(c, tag_id)
        c.showPage()
    c.save()

# =========================
# MAIN
# =========================
if __name__ == "__main__":
    # Шрифт с кириллицей (обычно есть на Linux)
    pdfmetrics.registerFont(TTFont("DejaVuSans", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"))
    pdfmetrics.registerFont(TTFont("DejaVuSans-Bold", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"))

    make_pdf(OUT_PDF, [10, 15, 20, 30])
    print("OK:", OUT_PDF)