import cv2
import numpy as np
from pathlib import Path

from roadseg import RoadSeg

# -----------------------------
# Настройки
# -----------------------------
INPUT_DIR  = Path("images")
MODEL_PATH = Path("roadseg_snow_best.onnx")
OUTPUT_DIR = Path("output_eval2")

# что сохраняем
SAVE_GRAY_MASK   = True   # mask_*.png (0..255)
SAVE_HEATMAP     = True   # heat_*.png (colormap)
SAVE_SIDE_BY_SIDE = True  # vis_*.png (orig | heatmap)

# colormap для heatmap (OpenCV)
# варианты: cv2.COLORMAP_TURBO, JET, INFERNO, VIRIDIS
COLORMAP = cv2.COLORMAP_TURBO

# -----------------------------
def to_u8(prob_mask: np.ndarray) -> np.ndarray:
    """float32 [0..1] -> uint8 [0..255]"""
    m = np.nan_to_num(prob_mask, nan=0.0, posinf=1.0, neginf=0.0).astype(np.float32)
    m = np.clip(m, 0.0, 1.0)
    return (m * 255.0).astype(np.uint8)

def main():
    if not INPUT_DIR.exists():
        raise FileNotFoundError(f"Папка с изображениями не найдена: {INPUT_DIR}")
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"ONNX-модель не найдена: {MODEL_PATH}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "gray").mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "heat").mkdir(parents=True, exist_ok=True)
    (OUTPUT_DIR / "vis").mkdir(parents=True, exist_ok=True)

    seg = RoadSeg(onnx_path=str(MODEL_PATH))
    if not seg.ok:
        print("[ERROR] RoadSeg не инициализирован. Проверь путь к onnx и логи выше.")
        return

    exts = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}
    image_paths = [p for p in INPUT_DIR.iterdir() if p.suffix.lower() in exts]
    image_paths = sorted(image_paths)

    if not image_paths:
        print(f"[WARNING] В {INPUT_DIR} нет изображений с расширениями {exts}")
        return

    print(f"[INFO] Найдено изображений: {len(image_paths)}")
    print(f"[INFO] Model input_size={seg.input_size} (W,H)")

    for img_path in image_paths:
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)  # BGR
        if img is None:
            print(f"[ERROR] Не удалось прочитать {img_path}, пропускаем.")
            continue

        # --- сырая вероятность [0..1], HxW float32
        prob = seg.infer(img)
        if prob is None or prob.size == 0:
            print(f"[WARNING] infer() вернул пусто для {img_path}, пропускаем.")
            continue

        # --- grayscale 0..255
        m_u8 = to_u8(prob)

        # 1) Сохраняем grayscale mask
        if SAVE_GRAY_MASK:
            out_gray = OUTPUT_DIR / "gray" / f"{img_path.stem}_mask.png"
            cv2.imwrite(str(out_gray), m_u8)

        # 2) Сохраняем heatmap
        if SAVE_HEATMAP or SAVE_SIDE_BY_SIDE:
            heat = cv2.applyColorMap(m_u8, COLORMAP)  # BGR

            if SAVE_HEATMAP:
                out_heat = OUTPUT_DIR / "heat" / f"{img_path.stem}_heat.png"
                cv2.imwrite(str(out_heat), heat)

            # 3) Side-by-side: original | heat
            if SAVE_SIDE_BY_SIDE:
                # приводим heat к размеру оригинала (на всякий)
                heat_rs = cv2.resize(heat, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
                vis = np.hstack([img, heat_rs])
                out_vis = OUTPUT_DIR / "vis" / f"{img_path.stem}_vis.png"
                cv2.imwrite(str(out_vis), vis)

        print(f"[OK] {img_path.name}")

    print(f"\n[DONE] Results in: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()