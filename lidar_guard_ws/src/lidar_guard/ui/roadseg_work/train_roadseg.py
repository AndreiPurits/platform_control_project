# -*- coding: utf-8 -*-
"""
roadseg_training.py

1) COCO -> PNG-маски для дороги из ДВУХ файлов:
   - result_road.json  : аннотации дороги ("road")
   - result_ignore.json: аннотации игнора ("ignore", "bucket", "kovsh")

   Маска:
     0   = фон
     1   = дорога
     255 = игнор (ковш, нежелательная область, нижняя полоска etc.)

2) Обучение маленького U-Net на этих масках:
   - вход: 384x384
   - выход: 1-канальная карта логитов
   - loss: BCEWithLogits по пикселям !=255
   - экспорт в ONNX

Структура директорий:
  BASE = /home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work

  BASE/result_road.json     -- COCO из Label Studio: класс "road"
  BASE/result_ignore.json   -- COCO из Label Studio: класс "ignore"/"kovsh"/"bucket"
  BASE/images/              -- исходные кадры

  BASE/asphalt/images/      -- будут скопированы кадры
  BASE/asphalt/masks/       -- сюда пишем PNG-маски (0/1/255)
"""

import os
import shutil
from pathlib import Path
from typing import List, Tuple, Dict

import cv2
import json
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader, random_split

# ------------------ ПУТИ И КОНФИГ ------------------

BASE = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work").resolve()

COCO_ROAD_JSON   = BASE / "result_road.json"
COCO_IGNORE_JSON = BASE / "result_ignore.json"

SRC_IMG_DIR = BASE / "images"

DATA_ROOT = BASE / "asphalt"
IMG_DIR   = DATA_ROOT / "images"
MASK_DIR  = DATA_ROOT / "masks"

IMG_SIZE   = (384, 384)   # (W, H)
VAL_RATIO  = 0.2
BATCH_SIZE = 4
EPOCHS     = 30
LR         = 1e-3
NUM_WORKERS = 4

IGNORE_LABEL = 255  # что пишем в маску для "ignore"


# ====================================================
#       HELPERS: ключ по "хвосту" имени и COCO merge
# ====================================================

def tail_key_from_name(basename: str) -> str:
    """
    Строим ключ по "хвосту" имени: от конца до первого '-'.

    Примеры:
      "18a2748d-357_76.png"              -> "357_76.png"
      "319d4106-18a2748d-357_76.png"     -> "357_76.png"
      "abc-def-123_45.png"               -> "123_45.png"

    Это позволяет сматчить road / ignore, даже если во втором
    COCO добавился лишний префикс.
    """
    name = Path(basename).name
    # идём с конца: находим последний '-'
    idx = name.rfind("-")
    if idx == -1:
        return name
    return name[idx + 1:]


def _collect_from_coco(
    coco_path: Path,
    which: str,
    images_by_tail: Dict[str, Dict]
):
    """
    which: 'road' или 'ignore'
    images_by_tail: dict[tail_key] = {
        'road':   [segmentation, ...],
        'ignore': [segmentation, ...],
        'road_basename': str | None,
        'basenames': set[str],   # все базовые имена, встреченные для этого tail
    }
    """
    if not coco_path.exists():
        print(f"[COCO] файл {coco_path} не найден, пропускаю {which}")
        return

    print(f"[COCO] читаю {which} из {coco_path}")
    with coco_path.open("r", encoding="utf-8") as f:
        coco = json.load(f)

    images = {img["id"]: img for img in coco.get("images", [])}
    anns   = coco.get("annotations", [])
    cats   = {c["id"]: c.get("name", "") for c in coco.get("categories", [])}

    n_used = 0

    for a in anns:
        img_id = a.get("image_id")
        if img_id is None or img_id not in images:
            continue

        cat_id = a.get("category_id")
        cat_name = cats.get(cat_id, "").strip().lower()

        if which == "road":
            if cat_name != "road":
                continue
        elif which == "ignore":
            if cat_name not in ("ignore", "bucket", "kovsh"):
                continue
        else:
            continue

        seg = a.get("segmentation")
        if not seg:
            continue

        img_info = images[img_id]
        basename_raw = Path(img_info.get("file_name", "")).name
        if not basename_raw:
            continue

        tail = tail_key_from_name(basename_raw)

        rec = images_by_tail.setdefault(
            tail,
            {
                "road": [],
                "ignore": [],
                "road_basename": None,
                "basenames": set(),
            }
        )
        rec[which].append(seg)
        rec["basenames"].add(basename_raw)
        if which == "road":
            rec["road_basename"] = basename_raw

        n_used += 1

    print(f"[COCO] {which}: использовано аннотаций: {n_used}")


def coco_to_roadmasks():
    """
    Собирает маски, используя ДВА COCO:

      - result_road.json   -> дорога (1)
      - result_ignore.json -> игнор (255, ковш и т.п.)

    Сопоставление картинок делается по "хвосту" имени
    (от конца до первого '-'), чтобы состыковать файлы вида:

      road  : 18a2748d-357_76.png
      ignore: 319d4106-18a2748d-357_76.png

    Логика рисования:
      - все полигоны "road" рисуются значением 1
      - все полигоны "ignore"/"bucket"/"kovsh" рисуются значением 255
        и ПЕРЕЗАТИРАЮТ дорогу
    """
    IMG_DIR.mkdir(parents=True, exist_ok=True)
    MASK_DIR.mkdir(parents=True, exist_ok=True)

    if not SRC_IMG_DIR.exists():
        print(f"[ERR] SRC images не найдены: {SRC_IMG_DIR}")
        return

    # собираем инфу по двум JSON по tail_key
    images_by_tail: Dict[str, Dict] = {}
    _collect_from_coco(COCO_ROAD_JSON,   "road",   images_by_tail)
    _collect_from_coco(COCO_IGNORE_JSON, "ignore", images_by_tail)

    if not images_by_tail:
        print("[COCO] Нет ни одной аннотации road/ignore, проверяй пути к JSON")
        return

    print(f"[COCO] всего разных tail-ключей (картинок): {len(images_by_tail)}")

    # предварительно соберём список всех картинок в SRC_IMG_DIR
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    all_src_images = [p for p in SRC_IMG_DIR.iterdir() if p.suffix.lower() in exts]

    def find_src_image(tail: str, preferred_basename: str | None):
        """
        Пытаемся найти реальный файл на диске:
          1) Если есть preferred_basename (из road COCO) -> берём его.
          2) Иначе ищем любой файл в SRC_IMG_DIR, чьё имя заканчивается на tail.
        """
        if preferred_basename:
            p = SRC_IMG_DIR / preferred_basename
            if p.exists():
                return p

        # fallback: поиск по хвосту
        for p in all_src_images:
            if p.name.endswith(tail):
                return p
        return None

    for tail, info in images_by_tail.items():
        basename = info.get("road_basename")
        src_path = find_src_image(tail, basename)

        if src_path is None:
            print(f"[WARN] картинка с tail='{tail}' не найдена в {SRC_IMG_DIR}, пропускаю")
            continue

        # копируем в asphalt/images
        dst_img = IMG_DIR / src_path.name
        if not dst_img.exists():
            shutil.copy2(src_path, dst_img)

        img = cv2.imread(str(src_path))
        if img is None:
            print(f"[WARN] не могу прочитать {src_path}, пропускаю")
            continue

        height, width = img.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)

        road_segs   = info.get("road", [])
        ignore_segs = info.get("ignore", [])

        def _draw_segmentation(segmentation, value: int):
            """
            segmentation: либо [x1,y1,...], либо [[...], [...], ...]
            """
            if isinstance(segmentation, list):
                if len(segmentation) > 0 and isinstance(segmentation[0], list):
                    polys = segmentation
                else:
                    polys = [segmentation]

                for poly in polys:
                    if len(poly) < 6:
                        continue
                    pts = np.array(poly, dtype=np.float32).reshape(-1, 2)
                    pts_int = np.round(pts).astype(np.int32)
                    cv2.fillPoly(mask, [pts_int], int(value))

        # 1) сначала дорога (1)
        for seg in road_segs:
            _draw_segmentation(seg, 1)

        # 2) затем ignore (255) перетирает дорогу
        for seg in ignore_segs:
            _draw_segmentation(seg, IGNORE_LABEL)

        stem = src_path.stem
        mask_path = MASK_DIR / f"{stem}.png"
        cv2.imwrite(str(mask_path), mask)
        print(
            f"[COCO] tail={tail} src={src_path.name} -> {mask_path.name} "
            f"(road_anns={len(road_segs)}, ignore_anns={len(ignore_segs)})"
        )

    print("\n[COCO] ГОТОВО. Маски в:", MASK_DIR)
    print("[COCO] Картинки в:", IMG_DIR)


# ====================================================
#                DATASET (картинка + маска)
# ====================================================

class RoadSegDataset(Dataset):
    """
    Возвращает:
      img_t  : float32 [3,H,W] в [0..1]
      mask_t : uint8   [H,W]   (0 фон, 1 дорога, 255 ignore)
    """
    def __init__(self, img_dir: Path, mask_dir: Path, img_size=(384, 384)):
        self.img_dir = Path(img_dir)
        self.mask_dir = Path(mask_dir)
        self.img_size = img_size

        exts = {".jpg", ".jpeg", ".png", ".bmp"}
        imgs = [p for p in self.img_dir.iterdir() if p.suffix.lower() in exts]

        self.items: List[Tuple[Path, Path]] = []
        for img in imgs:
            stem = img.stem
            mask = self.mask_dir / f"{stem}.png"
            if mask.exists():
                self.items.append((img, mask))

        if not self.items:
            raise RuntimeError(f"Не найдено пар image/mask в {self.img_dir} и {self.mask_dir}")

        print(f"[DATASET] пар: {len(self.items)}")

    def __len__(self):
        return len(self.items)

    def __getitem__(self, idx: int):
        img_path, mask_path = self.items[idx]

        # --- картинка ---
        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Не удалось прочитать {img_path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, self.img_size, interpolation=cv2.INTER_AREA)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # CHW

        # --- маска (0 фон, 1 дорога, 255 ignore) ---
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise RuntimeError(f"Не удалось прочитать {mask_path}")
        mask = cv2.resize(mask, self.img_size, interpolation=cv2.INTER_NEAREST)

        img_t  = torch.from_numpy(img)                    # float32 [3,H,W]
        mask_t = torch.from_numpy(mask.astype(np.uint8))  # uint8   [H,W]

        return img_t, mask_t


# ====================================================
#                    МАЛЕНЬКИЙ U-NET
# ====================================================

class DoubleConv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class UNetSmall(nn.Module):
    def __init__(self, in_ch=3, out_ch=1):
        super().__init__()
        self.down1 = DoubleConv(in_ch, 32)
        self.pool1 = nn.MaxPool2d(2)
        self.down2 = DoubleConv(32, 64)
        self.pool2 = nn.MaxPool2d(2)
        self.down3 = DoubleConv(64, 128)
        self.pool3 = nn.MaxPool2d(2)
        self.down4 = DoubleConv(128, 256)
        self.pool4 = nn.MaxPool2d(2)

        self.bottleneck = DoubleConv(256, 512)

        self.up4  = nn.ConvTranspose2d(512, 256, 2, stride=2)
        self.dec4 = DoubleConv(512, 256)
        self.up3  = nn.ConvTranspose2d(256, 128, 2, stride=2)
        self.dec3 = DoubleConv(256, 128)
        self.up2  = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.dec2 = DoubleConv(128, 64)
        self.up1  = nn.ConvTranspose2d(64, 32, 2, stride=2)
        self.dec1 = DoubleConv(64, 32)

        self.out_conv = nn.Conv2d(32, out_ch, 1)

    def forward(self, x):
        c1 = self.down1(x)
        p1 = self.pool1(c1)
        c2 = self.down2(p1)
        p2 = self.pool2(c2)
        c3 = self.down3(p2)
        p3 = self.pool3(c3)
        c4 = self.down4(p3)
        p4 = self.pool4(c4)

        b  = self.bottleneck(p4)

        u4 = self.up4(b)
        x4 = torch.cat([u4, c4], dim=1)
        d4 = self.dec4(x4)
        u3 = self.up3(d4)
        x3 = torch.cat([u3, c3], dim=1)
        d3 = self.dec3(x3)
        u2 = self.up2(d3)
        x2 = torch.cat([u2, c2], dim=1)
        d2 = self.dec2(x2)
        u1 = self.up1(d2)
        x1 = torch.cat([u1, c1], dim=1)
        d1 = self.dec1(x1)

        out = self.out_conv(d1)
        return out  # logits [B,1,H,W]


# ====================================================
#                 METRICS / TRAIN LOOP
# ====================================================

def iou_score(pred_logits, target_mask):
    """
    pred_logits: [B,1,H,W] (логиты)
    target_mask: uint8 [B,H,W] (0 фон, 1 дорога, 255 ignore)
    """
    with torch.no_grad():
        probs = torch.sigmoid(pred_logits).squeeze(1)  # [B,H,W]
        pred = (probs > 0.5).to(torch.uint8)

        tgt   = (target_mask == 1).to(torch.uint8)     # дорога
        valid = (target_mask != IGNORE_LABEL)

        pred = pred & valid
        tgt  = tgt & valid

        inter = (pred & tgt).sum().item()
        union = (pred | tgt).sum().item()
        if union == 0:
            return 1.0
        return inter / union


def train():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("[TRAIN] device:", device)

    ds = RoadSegDataset(IMG_DIR, MASK_DIR, img_size=IMG_SIZE)
    n_total = len(ds)
    n_val = max(1, int(n_total * VAL_RATIO))
    n_train = n_total - n_val
    train_ds, val_ds = random_split(ds, [n_train, n_val])
    print(f"[DATA] train={n_train}, val={n_val}")

    train_loader = DataLoader(
        train_ds,
        batch_size=BATCH_SIZE,
        shuffle=True,
        num_workers=NUM_WORKERS,
        pin_memory=True,
    )
    val_loader = DataLoader(
        val_ds,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=NUM_WORKERS,
        pin_memory=True,
    )

    model = UNetSmall(in_ch=3, out_ch=1).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    best_val_iou = 0.0
    ckpt_path = BASE / "roadseg_asphalt_best.pth"

    for epoch in range(1, EPOCHS + 1):
        model.train()
        running_loss = 0.0

        for imgs, masks in tqdm(train_loader, desc=f"Epoch {epoch}/{EPOCHS}"):
            imgs  = imgs.to(device, non_blocking=True)   # [B,3,H,W], float
            masks = masks.to(device, non_blocking=True)  # [B,H,W], uint8

            optimizer.zero_grad()
            logits = model(imgs)                         # [B,1,H,W]

            road  = (masks == 1).float()                # [B,H,W]
            valid = (masks != IGNORE_LABEL)             # [B,H,W]

            logits = logits.squeeze(1)                  # [B,H,W]

            logits_valid = logits[valid]
            road_valid   = road[valid]

            if logits_valid.numel() == 0:
                loss = torch.zeros((), device=device)
            else:
                loss = F.binary_cross_entropy_with_logits(logits_valid, road_valid)

            loss.backward()
            optimizer.step()

            running_loss += loss.item()

        avg_loss = running_loss / max(1, len(train_loader))
        print(f"[TRAIN] epoch {epoch}: loss={avg_loss:.4f}")

        # --- валидация ---
        model.eval()
        val_iou_acc = 0.0
        val_batches = 0

        with torch.no_grad():
            for imgs, masks in val_loader:
                imgs  = imgs.to(device, non_blocking=True)
                masks = masks.to(device, non_blocking=True)

                logits = model(imgs)
                iou = iou_score(logits, masks)
                val_iou_acc += iou
                val_batches += 1

        val_iou = val_iou_acc / max(1, val_batches)
        print(f"[VAL] epoch {epoch}: IoU={val_iou:.4f}")

        if val_iou > best_val_iou:
            best_val_iou = val_iou
            torch.save(
                {
                    "epoch": epoch,
                    "val_iou": best_val_iou,
                    "model_state": model.state_dict(),
                    "img_size": IMG_SIZE,
                },
                ckpt_path,
            )
            print(f"[CKPT] новый лучший IoU={best_val_iou:.4f} -> {ckpt_path}")

    print(f"[DONE] Обучение завершено. Лучший val_IoU={best_val_iou:.4f}")

    # -------- экспорт в ONNX --------
    try:
        onnx_path = BASE / "roadseg_asphalt.onnx"
        model.eval()
        dummy = torch.randn(1, 3, IMG_SIZE[1], IMG_SIZE[0], device=device)
        torch.onnx.export(
            model, dummy, str(onnx_path),
            input_names=["input"], output_names=["logits"],
            opset_version=11, dynamic_axes=None,
        )
        print(f"[ONNX] экспортировано в {onnx_path}")
    except Exception as e:
        print("[ONNX] экспорт не удался:", e)


# ====================================================
#                      MAIN
# ====================================================

if __name__ == "__main__":
    # Выбери режим:
    #   "coco"  – только конвертация COCO -> маски (из двух JSON)
    #   "train" – только обучение (предполагается, что маски уже готовы)
    #   "all"   – сначала coco_to_roadmasks(), затем train()
    mode = "all"

    if mode in ("coco", "all"):
        coco_to_roadmasks()
    if mode in ("train", "all"):
        train()