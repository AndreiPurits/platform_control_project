from pycocotools.coco import COCO
from pycocotools import mask as maskUtils  # Важно: импортируем mask
import cv2
import numpy as np
from pathlib import Path

COCO_JSON = "result.json"
IMG_DIR = Path("images")
MASK_DIR = Path("masks")
MASK_DIR.mkdir(exist_ok=True)

# Загружаем COCO-аннотации
coco = COCO(COCO_JSON)

# Получаем все изображения
img_ids = coco.getImgIds()

for img_id in img_ids:
    # Информация об изображении
    img_info = coco.loadImgs(img_id)[0]
    file_name = Path(img_info["file_name"]).name
    src_path = IMG_DIR / file_name

    if not src_path.exists():
        print(f"[WARN] Не найдено: {src_path}")
        continue

    # Загружаем изображение
    img = cv2.imread(str(src_path))
    if img is None:
        print(f"[ERROR] Не удалось загрузить: {src_path}")
        continue
    h, w = img.shape[:2]

    # Создаём пустую маску
    mask = np.zeros((h, w), dtype=np.uint8)

    # Получаем аннотации для изображения
    ann_ids = coco.getAnnIds(imgIds=img_id)
    anns = coco.loadAnns(ann_ids)

    for ann in anns:
        # Преобразуем segmentation в RLE (run-length encoding)
        rle = maskUtils.frPyObjects(ann["segmentation"], h, w)
        # Декодируем RLE в бинарную маску
        m = maskUtils.decode(rle)
        # Объединяем с общей маской
        mask = np.logical_or(mask, m[:, :, 0]).astype(np.uint8)  # Убираем лишнее измерение

    # Сохраняем маску (умножаем на 255 для визуализации)
    out_path = MASK_DIR / f"{file_name.replace('.jpg', '.png')}"
    cv2.imwrite(str(out_path), mask * 255)
    print(f"Сохранено: {out_path}")
