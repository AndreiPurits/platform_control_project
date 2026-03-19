
# -*- coding: utf-8 -*-
"""
train_roadseg_snow_ft.py

Fine-tune под доменный сдвиг (солнце/тени) при маленьком датасете:
- Всего ~253 (214 old + 39 new)
- VAL обязательно содержит "new" (например 15 новых + 25 старых)
- LR=1e-4, EPOCHS=40, early stopping
- Loss: 0.35*BCE + 0.65*Dice (как у тебя лучше работало)
- mIoU считается "по картинкам" (IoU на каждом изображении -> среднее)
- Сохраняет TOP-3 чекпоинта по val mIoU + best.pth и best.onnx

Ожидаемая структура данных (выбери один вариант):

Вариант A (рекомендую):
 DATA_ROOT/
   images_old/
   masks_old/
   images_new/
   masks_new/

Вариант B:
 DATA_ROOT/
   images/
   masks/
 и новые кадры имеют префикс "new_" или лежат по списку NEW_LIST_TXT.

Запуск:
 python3 train_roadseg_snow_ft.py
"""

import os
import random
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader


# =========================
# CONFIG
# =========================
SEED = 42
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

BASE = Path("/home/andrei/lidar_guard_ws/src/lidar_guard/ui/roadseg_work/ft").resolve()
DATA_ROOT = BASE
PRETRAINED_PTH = BASE / "roadseg_snow_best.pth"  # твоя старая модель
# --- ВЫБЕРИ ОДИН РЕЖИМ ДАННЫХ ---
USE_SPLIT_FOLDERS = True  # True -> images_old/images_new; False -> images/masks (+NEW_PREFIX/NEW_LIST_TXT)

IMG_OLD_DIR = DATA_ROOT / "images_old"
MSK_OLD_DIR = DATA_ROOT / "masks_old"
IMG_NEW_DIR = DATA_ROOT / "images_new"
MSK_NEW_DIR = DATA_ROOT / "masks_new"

IMG_DIR = DATA_ROOT / "images"
MSK_DIR = DATA_ROOT / "masks"
NEW_PREFIX = "new_"             # если USE_SPLIT_FOLDERS=False
NEW_LIST_TXT = DATA_ROOT / "new_list.txt"  # опционально: список имён новых кадров (без пути)

# --- split под твой кейс: 214 old + 39 new ---
VAL_NEW_N = 15
VAL_OLD_N = 25
VAL_RATIO_FALLBACK = 0.20  # если не получилось сделать вручную

# train
IMG_SIZE = (384, 384)  # (W,H)
BATCH_SIZE = 5
EPOCHS = 40
LR = 1e-4
NUM_WORKERS = 4

# loss
W_BCE = 0.35
W_DICE = 0.65
POS_WEIGHT_CLAMP = (1.0, 25.0)

# early stopping
PATIENCE = 7
MIN_DELTA = 1e-4

# aug
AUG_PROB = 0.95

# outputs
OUT_DIR = DATA_ROOT / "out_ft"
OUT_DIR.mkdir(parents=True, exist_ok=True)
CKPT_BEST = OUT_DIR / "roadseg_best.pth"
ONNX_BEST = OUT_DIR / "roadseg_best.onnx"


# =========================
# HELPERS
# =========================
def _clamp(x, lo, hi):
   return lo if x < lo else hi if x > hi else x

def _rand(a, b):
   return a + (b - a) * np.random.rand()

def list_images(img_dir: Path) -> List[Path]:
   exts = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
   if not img_dir.exists():
       return []
   return sorted([p for p in img_dir.iterdir() if p.suffix.lower() in exts])

def build_pairs(img_dir: Path, msk_dir: Path) -> List[Tuple[Path, Path]]:
   imgs = list_images(img_dir)
   out = []
   for ip in imgs:
       mp = msk_dir / f"{ip.stem}.png"
       if mp.exists():
           out.append((ip, mp))
   return out

def read_new_list(txt_path: Path) -> Optional[set]:
   if txt_path is None or (not txt_path.exists()):
       return None
   names = []
   for line in txt_path.read_text(encoding="utf-8").splitlines():
       line = line.strip()
       if not line:
           continue
       names.append(Path(line).stem)
   return set(names) if names else None


# =========================
# AUGMENT
# =========================
def add_motion_blur_rgb(img: np.ndarray, k: int) -> np.ndarray:
   k = int(_clamp(k, 3, 15))
   kernel = np.zeros((k, k), dtype=np.float32)
   kernel[:, k // 2] = 1.0
   kernel /= max(1e-6, kernel.sum())
   return cv2.filter2D(img, -1, kernel)

def add_random_shadow_rgb(img: np.ndarray) -> np.ndarray:
   h, w = img.shape[:2]
   x1, y1 = random.randint(0, w // 2), random.randint(0, h // 2)
   x2, y2 = random.randint(w // 2, w), random.randint(h // 2, h)
   shadow = np.zeros_like(img, dtype=np.uint8)
   cv2.rectangle(shadow, (x1, y1), (x2, y2), (50, 50, 50), -1)
   alpha = float(_rand(0.20, 0.45))
   return cv2.addWeighted(img, 1.0, shadow, alpha, 0.0)

def hsv_jitter_rgb(img: np.ndarray) -> np.ndarray:
   hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV).astype(np.float32)
   hsv[..., 1] *= float(_rand(0.55, 1.45))  # saturation
   hsv[..., 2] *= float(_rand(0.55, 1.55))  # value
   hsv = np.clip(hsv, 0, 255).astype(np.uint8)
   return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)

def augment_pair(img_rgb_u8: np.ndarray, mask_u8: np.ndarray, p: float = 0.95):
   if np.random.rand() > p:
       return img_rgb_u8, mask_u8

   img = img_rgb_u8.copy()
   m = mask_u8.copy()
   H, W = img.shape[:2]

   # flip
   if np.random.rand() < 0.5:
       img = img[:, ::-1, :]
       m = m[:, ::-1]

   # rot+scale
   if np.random.rand() < 0.7:
       ang = float(_rand(-7.0, 7.0))
       sc  = float(_rand(0.97, 1.03))
       M = cv2.getRotationMatrix2D((W / 2.0, H / 2.0), ang, sc)
       img = cv2.warpAffine(img, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT101)
       m   = cv2.warpAffine(m,   M, (W, H), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_REFLECT101)

   # brightness/contrast
   if np.random.rand() < 0.85:
       alpha = float(_rand(0.60, 1.60))
       beta  = float(_rand(-40, 40))
       img_f = img.astype(np.float32) * alpha + beta
       img = np.clip(img_f, 0, 255).astype(np.uint8)

   # gamma
   if np.random.rand() < 0.55:
       g = float(_rand(0.60, 1.60))
       lut = np.array([((i / 255.0) ** g) * 255.0 for i in range(256)], dtype=np.uint8)
       img = cv2.LUT(img, lut)

   # hsv jitter (супер важно для снега/солнца)
   if np.random.rand() < 0.65:
       img = hsv_jitter_rgb(img)

   # motion blur
   if np.random.rand() < 0.30:
       img = add_motion_blur_rgb(img, int(np.random.choice([3, 5, 7])))

   # shadow
   if np.random.rand() < 0.40:
       img = add_random_shadow_rgb(img)

   m = (m > 0).astype(np.uint8)
   return img, m


# =========================
# DATASET
# =========================
class RoadSegDataset(Dataset):
   def __init__(self, items: List[Tuple[Path, Path]], img_size=(384, 384), augment=False):
       self.items = list(items)
       self.img_size = tuple(img_size)  # (W,H)
       self.augment = bool(augment)
       if not self.items:
           raise RuntimeError("Dataset items is empty")

   def __len__(self):
       return len(self.items)

   def __getitem__(self, i: int):
       ip, mp = self.items[i]

       img_bgr = cv2.imread(str(ip), cv2.IMREAD_COLOR)
       if img_bgr is None:
           raise RuntimeError(f"Can't read image: {ip}")
       img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

       mask = cv2.imread(str(mp), cv2.IMREAD_GRAYSCALE)
       if mask is None:
           raise RuntimeError(f"Can't read mask: {mp}")
       mask = (mask > 0).astype(np.uint8)

       # resize (маска строго NEAREST!)
       Wm, Hm = self.img_size
       img_rgb = cv2.resize(img_rgb, (Wm, Hm), interpolation=cv2.INTER_AREA)
       mask = cv2.resize(mask, (Wm, Hm), interpolation=cv2.INTER_NEAREST)

       if self.augment:
           img_rgb, mask = augment_pair(img_rgb, mask, p=AUG_PROB)

       img = (img_rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)  # CHW
       mask = mask.astype(np.float32)  # HW 0/1
       return torch.from_numpy(img), torch.from_numpy(mask)


# =========================
# MODEL (UNetSmall)
# =========================
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
       d1 = self.up1(b); d1 = torch.cat([d1, e4], dim=1); d1 = self.dec1(d1)
       d2 = self.up2(d1); d2 = torch.cat([d2, e3], dim=1); d2 = self.dec2(d2)
       d3 = self.up3(d2); d3 = torch.cat([d3, e2], dim=1); d3 = self.dec3(d3)
       d4 = self.up4(d3); d4 = torch.cat([d4, e1], dim=1); d4 = self.dec4(d4)
       return self.out(d4)  # logits


# =========================
# LOSS / METRICS
# =========================
def dice_loss_with_logits(logits: torch.Tensor, target01: torch.Tensor, eps: float = 1e-6) -> torch.Tensor:
   probs = torch.sigmoid(logits)
   num = 2.0 * (probs * target01).sum(dim=(2, 3))
   den = (probs + target01).sum(dim=(2, 3)) + eps
   return (1.0 - (num / den)).mean()

def mean_iou_per_image_from_logits(logits: torch.Tensor, target01: torch.Tensor, thr: float = 0.5) -> float:
   """
   IoU считается по каждой картинке отдельно, затем усредняется.
   logits: [B,1,H,W]
   target01: [B,1,H,W]
   """
   with torch.no_grad():
       probs = torch.sigmoid(logits)
       pred = (probs > thr).to(torch.uint8)
       tgt  = (target01 > 0.5).to(torch.uint8)

       # per-image sums
       inter = (pred & tgt).flatten(1).sum(dim=1).cpu().numpy().astype(np.float32)
       union = (pred | tgt).flatten(1).sum(dim=1).cpu().numpy().astype(np.float32)

       iou = np.where(union <= 0.0, 1.0, inter / np.maximum(1.0, union))
       return float(iou.mean())

def estimate_pos_weight(items: List[Tuple[Path, Path]], img_size, max_items: int = 300) -> float:
   """
   pos_weight = neg/pos для BCE. Считаем по train маскам после resize.
   """
   Wm, Hm = img_size
   pos = 0
   neg = 0
   n = min(len(items), int(max_items))
   for i in range(n):
       _, mp = items[i]
       m = cv2.imread(str(mp), cv2.IMREAD_GRAYSCALE)
       if m is None:
           continue
       m = (m > 0).astype(np.uint8)
       m = cv2.resize(m, (Wm, Hm), interpolation=cv2.INTER_NEAREST)
       pos += int((m > 0).sum())
       neg += int((m == 0).sum())
   pos = max(1, pos)
   return float(neg) / float(pos)


# =========================
# SPLIT
# =========================
def split_items_stratified(old_items, new_items, val_old_n, val_new_n):
   random.shuffle(old_items)
   random.shuffle(new_items)

   val_old = old_items[:min(val_old_n, len(old_items))]
   val_new = new_items[:min(val_new_n, len(new_items))]

   train_old = old_items[len(val_old):]
   train_new = new_items[len(val_new):]

   val = val_old + val_new
   train = train_old + train_new
   random.shuffle(train)
   random.shuffle(val)
   return train, val

def split_fallback(items, val_ratio=0.2):
   items = list(items)
   random.shuffle(items)
   k = int(len(items) * val_ratio)
   return items[k:], items[:k]


# =========================
# TOP-K CHECKPOINTS
# =========================
class TopK:
   def __init__(self, k=3):
       self.k = int(k)
       self.buf = []  # list of (score, path)

   def add(self, score: float, path: Path):
       self.buf.append((float(score), Path(path)))
       self.buf.sort(key=lambda t: t[0], reverse=True)
       # remove extra
       while len(self.buf) > self.k:
           _, p = self.buf.pop(-1)
           try:
               if p.exists():
                   p.unlink()
           except Exception:
               pass

   def best(self):
       return self.buf[0] if self.buf else None


# =========================
# TRAIN / EVAL
# =========================
def train_one_epoch(model, loader, optimizer, pos_weight_t, device):
   model.train()
   tot_loss = 0.0
   tot_miou = 0.0
   n = 0

   for imgs, masks in tqdm(loader, desc="Train", leave=False):
       imgs = imgs.to(device, non_blocking=True)
       masks01 = masks.to(device, non_blocking=True).unsqueeze(1)

       optimizer.zero_grad()
       logits = model(imgs)

       bce = F.binary_cross_entropy_with_logits(logits, masks01, pos_weight=pos_weight_t)
       dice = dice_loss_with_logits(logits, masks01)
       loss = W_BCE * bce + W_DICE * dice

       loss.backward()
       optimizer.step()

       tot_loss += float(loss.item())
       tot_miou += mean_iou_per_image_from_logits(logits, masks01, thr=0.5)
       n += 1

   return tot_loss / max(1, n), tot_miou / max(1, n)

@torch.no_grad()
def eval_one_epoch(model, loader, pos_weight_t, device):
   model.eval()
   tot_loss = 0.0
   tot_miou = 0.0
   n = 0

   for imgs, masks in tqdm(loader, desc="Val", leave=False):
       imgs = imgs.to(device, non_blocking=True)
       masks01 = masks.to(device, non_blocking=True).unsqueeze(1)
       logits = model(imgs)

       bce = F.binary_cross_entropy_with_logits(logits, masks01, pos_weight=pos_weight_t)
       dice = dice_loss_with_logits(logits, masks01)
       loss = W_BCE * bce + W_DICE * dice

       tot_loss += float(loss.item())
       tot_miou += mean_iou_per_image_from_logits(logits, masks01, thr=0.5)
       n += 1

   return tot_loss / max(1, n), tot_miou / max(1, n)


def main():
   device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
   print("[DEV]", device)

   # --- gather items ---
   if USE_SPLIT_FOLDERS:
       old_items = build_pairs(IMG_OLD_DIR, MSK_OLD_DIR)
       new_items = build_pairs(IMG_NEW_DIR, MSK_NEW_DIR)
       all_items = old_items + new_items
       print(f"[DATA] old={len(old_items)} new={len(new_items)} total={len(all_items)}")
       if len(all_items) < 10:
           raise RuntimeError("Too few items. Check folders.")

       train_items, val_items = split_items_stratified(old_items, new_items, VAL_OLD_N, VAL_NEW_N)
       if len(val_items) < 5:
           print("[WARN] stratified split failed -> fallback random split")
           train_items, val_items = split_fallback(all_items, VAL_RATIO_FALLBACK)

   else:
       all_items = build_pairs(IMG_DIR, MSK_DIR)
       new_set = read_new_list(NEW_LIST_TXT)
       old_items, new_items = [], []
       for ip, mp in all_items:
           if new_set is not None:
               (new_items if ip.stem in new_set else old_items).append((ip, mp))
           else:
               (new_items if ip.name.startswith(NEW_PREFIX) else old_items).append((ip, mp))

       print(f"[DATA] old={len(old_items)} new={len(new_items)} total={len(all_items)}")
       if len(new_items) == 0:
           print("[WARN] new_items==0 -> fallback random split")
           train_items, val_items = split_fallback(all_items, VAL_RATIO_FALLBACK)
       else:
           train_items, val_items = split_items_stratified(old_items, new_items, VAL_OLD_N, VAL_NEW_N)

   print(f"[SPLIT] train={len(train_items)} val={len(val_items)}")
   if len(val_items) < 10:
       print("[WARN] val too small; consider increasing VAL_OLD_N/VAL_NEW_N")

   # datasets
   train_ds = RoadSegDataset(train_items, img_size=IMG_SIZE, augment=True)
   val_ds   = RoadSegDataset(val_items, img_size=IMG_SIZE, augment=False)

   train_loader = DataLoader(
       train_ds, batch_size=BATCH_SIZE, shuffle=True,
       num_workers=NUM_WORKERS, pin_memory=True, drop_last=False
   )
   val_loader = DataLoader(
       val_ds, batch_size=BATCH_SIZE, shuffle=False,
       num_workers=NUM_WORKERS, pin_memory=True, drop_last=False
   )

   model = UNetSmall(in_channels=3, out_channels=1).to(device)

   if PRETRAINED_PTH.exists():
       ckpt = torch.load(PRETRAINED_PTH, map_location=device)
       state = ckpt["model_state"] if "model_state" in ckpt else ckpt
       model.load_state_dict(state, strict=True)
       print(f"[INIT] loaded pretrained weights: {PRETRAINED_PTH}")
   else:
      print("[INIT] pretrained not found, training from scratch")

   # pos_weight estimated once (как у тебя)
   pos_w = estimate_pos_weight(train_items, IMG_SIZE, max_items=300)
   pos_w = float(_clamp(pos_w, POS_WEIGHT_CLAMP[0], POS_WEIGHT_CLAMP[1]))
   pos_weight_t = torch.tensor([pos_w], device=device)
   print(f"[LOSS] W_BCE={W_BCE} W_DICE={W_DICE} pos_weight={pos_w:.2f}")

   optimizer = torch.optim.Adam(model.parameters(), lr=LR)

   best_val = -1e9
   patience_left = PATIENCE

   topk = TopK(k=3)

   for epoch in range(1, EPOCHS + 1):
       tr_loss, tr_miou = train_one_epoch(model, train_loader, optimizer, pos_weight_t, device)
       va_loss, va_miou = eval_one_epoch(model, val_loader, pos_weight_t, device)

       print(
           f"[E{epoch:03d}] "
           f"train: loss={tr_loss:.4f} mIoU={tr_miou:.4f} | "
           f"val:   loss={va_loss:.4f} mIoU={va_miou:.4f}"
       )

       # save top-3
       ckpt_path = OUT_DIR / f"roadseg_epoch{epoch:03d}_miou{va_miou:.4f}.pth"
       torch.save(
           {
               "epoch": epoch,
               "val_miou": float(va_miou),
               "model_state": model.state_dict(),
               "img_size": IMG_SIZE,
               "pos_weight": float(pos_w),
               "loss_w": (W_BCE, W_DICE),
           },
           ckpt_path,
       )
       topk.add(va_miou, ckpt_path)

       # early stopping
       if va_miou > (best_val + MIN_DELTA):
           best_val = float(va_miou)
           patience_left = PATIENCE
           # copy best
           torch.save(
               {
                   "epoch": epoch,
                   "val_miou": float(best_val),
                   "model_state": model.state_dict(),
                   "img_size": IMG_SIZE,
                   "pos_weight": float(pos_w),
                   "loss_w": (W_BCE, W_DICE),
               },
               CKPT_BEST,
           )
           print(f"[BEST] new best val mIoU={best_val:.4f} -> {CKPT_BEST}")
       else:
           patience_left -= 1
           print(f"[ES] no improv. patience_left={patience_left}/{PATIENCE} best={best_val:.4f}")
           if patience_left <= 0:
               print("[ES] Early stopping.")
               break

   # export best to ONNX
   if CKPT_BEST.exists():
       ck = torch.load(CKPT_BEST, map_location="cpu")
       model.load_state_dict(ck["model_state"])
       model = model.cpu().eval()
       dummy = torch.randn(1, 3, IMG_SIZE[1], IMG_SIZE[0]).cpu()  # (N,C,H,W), IMG_SIZE=(W,H)
       torch.onnx.export(
           model, dummy, str(ONNX_BEST),
           input_names=["input"], output_names=["logits"],
           export_params=True, do_constant_folding=True,
           dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
       )
       print(f"[ONNX] exported -> {ONNX_BEST}")

   print("[TOP3]")
   for s, p in topk.buf:
       print(f"  {s:.4f}  {p.name}")
   print("[DONE]")


if __name__ == "__main__":
   main()