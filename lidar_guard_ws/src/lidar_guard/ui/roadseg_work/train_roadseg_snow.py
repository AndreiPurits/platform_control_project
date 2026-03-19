# -*- coding: utf-8 -*-
"""
train_roadseg_snow.py

Нормальный тренинг бинарной сегментации дороги (road vs bg) для малого датасета.

Фичи:
- корректные аугментации (реально применяются)
- корректный resize маски (INTER_NEAREST)
- mIoU = mean IoU по картинкам
- pos_weight (neg/pos) считается автоматически по train
- loss = w_bce * BCEWithLogits + w_dice * Dice (веса подобраны под твой кейс)
- early stopping
- сохранение лучшего чекпоинта + экспорт ONNX
"""

from __future__ import annotations

import os
import random
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

# ------------------ ПУТИ И КОНФИГ ------------------

BASE = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work").resolve()

DATA_ROOT = BASE
IMG_DIR   = DATA_ROOT / "images"
MASK_DIR  = DATA_ROOT / "masks"

# ВАЖНО: здесь используем ПОРЯДОК (W, H) — как у тебя в датасете и в RoadSeg input_size
IMG_SIZE_WH  = (384, 384)  # (W, H)

VAL_RATIO    = 0.20
BATCH_SIZE   = 6
EPOCHS       = 120
LR           = 1e-3
NUM_WORKERS  = 4

SEED = 42
AUG_PROB = 0.90

# веса лосса (ты говорил, что при 0.3-0.4 было лучше)
W_BCE  = 0.35
W_DICE = 0.65

# pos_weight clamp
POS_W_MIN = 1.0
POS_W_MAX = 40.0

# early stopping
PATIENCE = 8
MIN_DELTA = 1e-4

# --------------------------------------------
#                  SEED
# --------------------------------------------
def seed_everything(seed: int = 42) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)

# ====================================================
#                AUGMENTATIONS
# ====================================================

def _rand(a, b):
    return a + (b - a) * np.random.rand()

def add_motion_blur(img: np.ndarray, k: int) -> np.ndarray:
    k = int(max(3, k))
    if k % 2 == 0:
        k += 1
    kernel = np.zeros((k, k), dtype=np.float32)
    kernel[:, k // 2] = 1.0
    kernel /= kernel.sum()
    return cv2.filter2D(img, -1, kernel)

def add_random_shadow(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    x1, y1 = random.randint(0, w // 2), random.randint(0, h // 2)
    x2, y2 = random.randint(w // 2, w), random.randint(h // 2, h)
    shadow = np.zeros_like(img, dtype=np.uint8)
    cv2.rectangle(shadow, (x1, y1), (x2, y2), (40, 40, 40), -1)
    alpha = float(_rand(0.20, 0.40))
    return cv2.addWeighted(img, 1.0, shadow, alpha, 0.0)

def augment_pair(img_rgb_u8: np.ndarray, mask01_u8: np.ndarray, p: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    img: RGB uint8, HxWx3
    mask: uint8 0/1, HxW
    """
    if np.random.rand() > p:
        return img_rgb_u8, mask01_u8

    H, W = img_rgb_u8.shape[:2]
    img = img_rgb_u8.copy()
    m = mask01_u8.copy()

    # flip
    if np.random.rand() < 0.5:
        img = img[:, ::-1, :]
        m = m[:, ::-1]

    # small rotation + scale
    if np.random.rand() < 0.70:
        ang = _rand(-7.0, 7.0)
        sc  = _rand(0.98, 1.03)
        M = cv2.getRotationMatrix2D((W / 2.0, H / 2.0), ang, sc)
        img = cv2.warpAffine(img, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
        m   = cv2.warpAffine(m,   M, (W, H), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REFLECT101)

    # brightness/contrast
    if np.random.rand() < 0.80:
        alpha = _rand(0.70, 1.35)
        beta  = _rand(-30, 30)
        img_f = img.astype(np.float32) * alpha + beta
        img = np.clip(img_f, 0, 255).astype(np.uint8)

    # gamma
    if np.random.rand() < 0.50:
        g = _rand(0.70, 1.40)
        lut = np.array([((i / 255.0) ** g) * 255.0 for i in range(256)], dtype=np.uint8)
        img = cv2.LUT(img, lut)

    # blur
    if np.random.rand() < 0.35:
        k = int(np.random.choice([3, 5, 7]))
        img = add_motion_blur(img, k)

    # shadow
    if np.random.rand() < 0.25:
        img = add_random_shadow(img)

    m = (m > 0).astype(np.uint8)
    return img, m

#=============================
#                DATASET
# ====================================================

class RoadSegDataset(Dataset):
    def __init__(self, items: List[Tuple[Path, Path]], img_size_wh=(384, 384), augment: bool = False):
        self.items = list(items)
        self.W, self.H = int(img_size_wh[0]), int(img_size_wh[1])
        self.augment = bool(augment)

        if not self.items:
            raise RuntimeError("Dataset items is empty")

    def __len__(self):
        return len(self.items)

    def __getitem__(self, i: int):
        img_path, mask_path = self.items[i]

        img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError(f"Не удалось прочитать {img_path}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            raise RuntimeError(f"Не удалось прочитать {mask_path}")
        mask = (mask > 0).astype(np.uint8)  # 0/1

        # resize (OpenCV expects dsize=(W,H))
        img  = cv2.resize(img,  (self.W, self.H), interpolation=cv2.INTER_AREA)
        mask = cv2.resize(mask, (self.W, self.H), interpolation=cv2.INTER_NEAREST)

        # augment (на uint8!)
        if self.augment:
            img, mask = augment_pair(img, mask, p=AUG_PROB)

        # to float tensors
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))      # CHW
        mask = mask.astype(np.float32)          # HW 0/1

        return torch.from_numpy(img), torch.from_numpy(mask)

def build_items(img_dir: Path, mask_dir: Path) -> List[Tuple[Path, Path]]:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    imgs = [p for p in img_dir.iterdir() if p.suffix.lower() in exts]
    imgs = sorted(imgs)

    items = []
    for img in imgs:
        m = mask_dir / f"{img.stem}.png"
        if m.exists():
            items.append((img, m))

    if not items:
        raise RuntimeError(f"Не найдено пар image/mask в {img_dir} и {mask_dir}")
    return items

def simple_train_val_split(items: List[Tuple[Path, Path]], val_ratio: float, seed: int) -> Tuple[List[Tuple[Path, Path]], List[Tuple[Path, Path]]]:
    items = list(items)
    rnd = random.Random(seed)
    rnd.shuffle(items)
    split_idx = int(len(items) * (1 - val_ratio))
    train_items = items[:split_idx]
    val_items = items[split_idx:]
    print(f"[SPLIT] train={len(train_items)} val={len(val_items)} (val_ratio={val_ratio})")
    return train_items, val_items

# ====================================================
#                 MODEL
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
    def __init__(self, in_channels=3, out_channels=1):
        super().__init__()
        self.enc1 = DoubleConv(in_channels, 32)
        self.enc2 = DoubleConv(32, 64)
        self.enc3 = DoubleConv(64, 128)
        self.enc4 = DoubleConv(128, 256)
        self.bottleneck = DoubleConv(256, 512)

        self.up1 = nn.ConvTranspose2d(512, 256, 2, stride=2)
        self.dec1 = DoubleConv(512, 256)
        self.up2 = nn.ConvTranspose2d(256, 128, 2, stride=2)
        self.dec2 = DoubleConv(256, 128)
        self.up3 = nn.ConvTranspose2d(128, 64, 2, stride=2)
        self.dec3 = DoubleConv(128, 64)
        self.up4 = nn.ConvTranspose2d(64, 32, 2, stride=2)
        self.dec4 = DoubleConv(64, 32)

        self.out = nn.Conv2d(32, out_channels, 1)

    def forward(self, x):
        e1 = self.enc1(x)
        e2 = self.enc2(F.max_pool2d(e1, 2))
        e3 = self.enc3(F.max_pool2d(e2, 2))
        e4 = self.enc4(F.max_pool2d(e3, 2))
        b = self.bottleneck(F.max_pool2d(e4, 2))

        d1 = self.up1(b)
        d1 = torch.cat([d1, e4], dim=1)
        d1 = self.dec1(d1)

        d2 = self.up2(d1)
        d2 = torch.cat([d2, e3], dim=1)
        d2 = self.dec2(d2)

        d3 = self.up3(d2)
        d3 = torch.cat([d3, e2], dim=1)
        d3 = self.dec3(d3)

        d4 = self.up4(d3)
        d4 = torch.cat([d4, e1], dim=1)
        d4 = self.dec4(d4)

        return self.out(d4)  # logits [B,1,H,W]

# ====================================================
#                 LOSS & METRICS
# ====================================================

def dice_loss_with_logits(logits: torch.Tensor, target01: torch.Tensor, eps: float = 1e-6) -> torch.Tensor:
    probs = torch.sigmoid(logits)
    num = 2.0 * (probs * target01).sum(dim=(2, 3))
    den = (probs + target01).sum(dim=(2, 3)) + eps
    return (1.0 - (num / den)).mean()

def mean_iou_from_logits(logits: torch.Tensor, target01: torch.Tensor, thr: float = 0.5, eps: float = 1e-6) -> float:
    with torch.no_grad():
        probs = torch.sigmoid(logits)
        pred = (probs > thr).to(torch.uint8)
        tgt  = (target01 > 0.5).to(torch.uint8)

        inter = (pred & tgt).sum(dim=(1,2,3)).float()  # [B]
        union = (pred | tgt).sum(dim=(1,2,3)).float()  # [B]

        iou = torch.where(union > 0, inter / (union + eps), torch.ones_like(union))
        return float(iou.mean().item())

def estimate_pos_weight(train_items: List[Tuple[Path, Path]], img_size_wh: Tuple[int,int], max_items: int = 300) -> float:
    """
    pos_weight = neg/pos после resize маски под IMG_SIZE.
    img_size_wh=(W,H)
    """
    W, H = img_size_wh
    pos = 0
    neg = 0
    n = min(len(train_items), int(max_items))
    for i in range(n):
        _, mpath = train_items[i]
        m = cv2.imread(str(mpath), cv2.IMREAD_GRAYSCALE)
        if m is None:
            continue
        m = (m > 0).astype(np.uint8)
        m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
        pos += int((m > 0).sum())
        neg += int((m == 0).sum())
    pos = max(1, pos)
    return float(neg) / float(pos)

# ====================================================
#                 TRAIN / VAL
# ====================================================

def train_one_epoch(model, loader, optimizer, pos_weight_t, device) -> Tuple[float, float]:
    model.train()
    total_loss = 0.0
    total_miou = 0.0
    n_batches = 0

    for imgs, masks in tqdm(loader, desc="Train"):
        imgs = imgs.to(device, non_blocking=True)            # [B,3,H,W]
        masks01 = masks.to(device, non_blocking=True).unsqueeze(1)  # [B,1,H,W]

        optimizer.zero_grad()
        logits = model(imgs)

        bce = F.binary_cross_entropy_with_logits(logits, masks01, pos_weight=pos_weight_t)
        dice = dice_loss_with_logits(logits, masks01)
        loss = W_BCE * bce + W_DICE * dice

        loss.backward()
        optimizer.step()

        total_loss += float(loss.item())
        total_miou += mean_iou_from_logits(logits, masks01, thr=0.5)
        n_batches += 1

    return total_loss / max(1, n_batches), total_miou / max(1, n_batches)

@torch.no_grad()
def val_one_epoch(model, loader, pos_weight_t, device) -> Tuple[float, float]:
    model.eval()
    total_loss = 0.0
    total_miou = 0.0
    n_batches = 0

    for imgs, masks in tqdm(loader, desc="Val"):
        imgs = imgs.to(device, non_blocking=True)
        masks01 = masks.to(device, non_blocking=True).unsqueeze(1)

        logits = model(imgs)

        bce = F.binary_cross_entropy_with_logits(logits, masks01, pos_weight=pos_weight_t)
        dice = dice_loss_with_logits(logits, masks01)
        loss = W_BCE * bce + W_DICE * dice

        total_loss += float(loss.item())
        total_miou += mean_iou_from_logits(logits, masks01, thr=0.5)
        n_batches += 1

    return total_loss / max(1, n_batches), total_miou / max(1, n_batches)

def export_onnx(model: nn.Module, out_path: Path, img_size_wh: Tuple[int,int], device: torch.device) -> None:
    model.eval()
    W, H = img_size_wh
    dummy = torch.randn(1, 3, H, W, device=device)  # NCHW (H,W!)
    torch.onnx.export(
        model,
        dummy,
        str(out_path),
        input_names=["input"],
        output_names=["logits"],
        opset_version=11,
        dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
    )

# ====================================================
#                 MAIN
# ====================================================

def main():
    seed_everything(SEED)

    if not IMG_DIR.exists() or not MASK_DIR.exists():
        raise FileNotFoundError(f"Не найдены папки IMG_DIR={IMG_DIR} или MASK_DIR={MASK_DIR}")

    all_items = build_items(IMG_DIR, MASK_DIR)
    print(f"[DATA] total pairs: {len(all_items)}")

    train_items, val_items = simple_train_val_split(all_items, VAL_RATIO, SEED)

    # pos_weight автооценка
    pos_w_est = estimate_pos_weight(train_items, IMG_SIZE_WH, max_items=300)
    pos_w = float(np.clip(pos_w_est, POS_W_MIN, POS_W_MAX))
    print(f"[POS_W] estimated={pos_w_est:.4f} -> used(clamped)={pos_w:.4f}")

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[DEVICE] {device}")

    # datasets/loaders
    train_ds = RoadSegDataset(train_items, img_size_wh=IMG_SIZE_WH, augment=True)
    val_ds   = RoadSegDataset(val_items,   img_size_wh=IMG_SIZE_WH, augment=False)

    train_loader = DataLoader(
        train_ds, batch_size=BATCH_SIZE, shuffle=True,
        num_workers=NUM_WORKERS, pin_memory=True, drop_last=False
    )
    val_loader = DataLoader(
        val_ds, batch_size=BATCH_SIZE, shuffle=False,
        num_workers=NUM_WORKERS, pin_memory=True, drop_last=False
    )

    # model/optim
    model = UNetSmall(in_channels=3, out_channels=1).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)

    pos_weight_t = torch.tensor([pos_w], dtype=torch.float32, device=device)

    best_val_miou = -1.0
    best_epoch = -1
    ckpt_path = BASE / "roadseg_snow_best.pth"

    # early stopping state
    patience_left = PATIENCE
    last_best = -1.0

    print(f"[LOSS] W_BCE={W_BCE}  W_DICE={W_DICE}  (sum={W_BCE+W_DICE})")

    for epoch in range(1, EPOCHS + 1):
        print(f"\n===== Epoch {epoch}/{EPOCHS} =====")
        tr_loss, tr_miou = train_one_epoch(model, train_loader, optimizer, pos_weight_t, device)
        va_loss, va_miou = val_one_epoch(model, val_loader, pos_weight_t, device)

        print(f"[TRAIN] loss={tr_loss:.4f}  mIoU={tr_miou:.4f}")
        print(f"[VAL]   loss={va_loss:.4f}  mIoU={va_miou:.4f}")

        # best checkpoint
        if va_miou > best_val_miou + MIN_DELTA:
            best_val_miou = va_miou
            best_epoch = epoch
            patience_left = PATIENCE

            torch.save(
                {
                    "epoch": best_epoch,
                    "val_miou": best_val_miou,
                    "model_state": model.state_dict(),
                    "img_size_wh": IMG_SIZE_WH,
                    "pos_weight": pos_w,
                    "loss_weights": {"bce": W_BCE, "dice": W_DICE},
                },
                ckpt_path,
            )
            print(f"[CKPT] saved best: mIoU={best_val_miou:.4f} epoch={best_epoch} -> {ckpt_path}")
            last_best = best_val_miou
        else:
            patience_left -= 1
            print(f"[ES] no improv. patience_left={patience_left}/{PATIENCE}  best={last_best:.4f}")
            if patience_left <= 0:
                print("[ES] Early stopping.")
                break

    print(f"\n[DONE] best val mIoU={best_val_miou:.4f} at epoch={best_epoch}")

    # export ONNX from best checkpoint
    try:
        ckpt = torch.load(str(ckpt_path), map_location=device)
        model.load_state_dict(ckpt["model_state"])
        onnx_path = BASE / "roadseg_snow_best.onnx"
        export_onnx(model, onnx_path, IMG_SIZE_WH, device=device)
        print(f"[ONNX] exported: {onnx_path}")
    except Exception as e:
        print("[ONNX] export failed:", e)

if __name__ == "__main__":
    main() 