
import os
import cv2
import numpy as np
from pathlib import Path
import onnxruntime as ort

# Настройки
INPUT_DIR = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work/datasets/photos/Poly_asf")
MODEL_PATH = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work/roadseg_model_best_posw_20.0.onnx")
OUTPUT_DIR = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work/output_collage")

MASK_COLOR = (0, 255, 0)  # Зеленый цвет маски
ALPHA = 0.6  # Прозрачность маски
THRESHOLD = 0.5  # Порог бинаризации

def apply_mask_to_image(image: np.ndarray, mask: np.ndarray, color: tuple, alpha: float) -> np.ndarray:
    # Убедимся, что маска имеет правильный тип и размер
    if mask.dtype != np.uint8:
        mask = (mask > THRESHOLD).astype(np.uint8)
    
    # Проверим размеры изображения и маски
    assert image.shape[:2] == mask.shape, "Размеры изображения и маски не совпадают"
    
    color_mask = np.zeros_like(image, dtype=np.uint8)
    color_mask[mask == 1] = color
    return cv2.addWeighted(image, 1.0, color_mask, alpha, 0.0)

def make_collage(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    h1, w1 = left.shape[:2]
    h2, w2 = right.shape[:2]
    if h1 != h2:
        right = cv2.resize(right, (w2, h1), interpolation=cv2.INTER_AREA)
    return np.hstack([left, right])

def main():
    if not INPUT_DIR.exists():
        raise FileNotFoundError(f"Папка с изображениями не найдена: {INPUT_DIR}")
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"ONNX-модель не найдена: {MODEL_PATH}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Инициализация ONNX сессии
    session = ort.InferenceSession(str(MODEL_PATH))
    input_name = session.get_inputs()[0].name
    
    # Собираем изображения
    exts = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}
    image_paths = [p for p in INPUT_DIR.iterdir() if p.suffix.lower() in exts]
    
    if not image_paths:
        print(f"[WARNING] В {INPUT_DIR} нет изображений с расширениями {exts}")
        return

    print(f"[INFO] Найдено изображений: {len(image_paths)}")

    for img_path in image_paths:
        try:
            img = cv2.imread(str(img_path))
            if img is None:
                print(f"[ERROR] Не удалось прочитать {img_path}, пропускаем.")
                continue

            # Препроцессинг изображения
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, (384, 384))  # Размер должен соответствовать модели
            img = img.transpose((2, 0, 1))  # CHW
            img = img.astype(np.float32) / 255.0
            img = np.expand_dims(img, 0)  # Добавить batch dimension

            # Инференс
            outputs = session.run(None, {input_name: img})
            mask_prob = np.squeeze(outputs[0], axis=(0, 1))  # HxW

            # Создаем изображение с маской
            img_bgr = cv2.cvtColor(img[0].transpose((1, 2, 0)) * 255, cv2.COLOR_RGB2BGR)
            # Создаем изображение с маской
            img_with_mask = apply_mask_to_image(img_bgr, mask_prob, MASK_COLOR, ALPHA)
            
            # Делаем коллаж: слева — оригинал, справа — с маской
            collage = make_collage(img_bgr, img_with_mask)
            
            # Сохраняем результат
            output_path = OUTPUT_DIR / img_path.name
            cv2.imwrite(str(output_path), collage)
            print(f"[OK] Сохранено: {output_path}")

        except Exception as e:
            print(f"[ERROR] Ошибка при обработке {img_path}: {e}")

        print(f"\n[DONE] Обработка завершена. Результаты в: {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
