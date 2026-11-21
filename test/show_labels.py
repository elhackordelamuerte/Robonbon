#!/usr/bin/env python3
"""Affiche les images d'un split YOLO avec leurs boîtes et noms de classes.

Usage :
    python test/show_labels.py                     # split test, toutes classes
    python test/show_labels.py --cls 8             # seulement les images qui ont la classe 8, et seulement ses boîtes
    python test/show_labels.py --split valid --cls 2 3

Touches : n / espace = suivante, p = précédente, q / Échap = quitter
"""
import argparse
from pathlib import Path

import cv2
import yaml

COLORS = [
    (0, 255, 255), (255, 0, 255), (0, 0, 255), (0, 128, 255),
    (0, 255, 0), (255, 255, 0), (255, 0, 0), (128, 0, 255),
    (0, 200, 128), (200, 128, 0), (128, 128, 255), (60, 60, 200),
]


def load_labels(label_path):
    """Retourne une liste de (cls, cx, cy, w, h) normalisés."""
    if not label_path.exists():
        return []
    boxes = []
    for line in label_path.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 5:
            boxes.append((int(parts[0]), *map(float, parts[1:5])))
    return boxes


def draw(img, boxes, names):
    h, w = img.shape[:2]
    for cls, cx, cy, bw, bh in boxes:
        x1, y1 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
        x2, y2 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
        color = COLORS[cls % len(COLORS)]
        label = f"{cls}: {names.get(cls, '?')}"
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
    return img


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="dataset/data.yaml")
    parser.add_argument("--split", default="test", choices=["train", "val", "valid", "test"])
    parser.add_argument("--cls", type=int, nargs="*", help="classes à afficher (défaut : toutes)")
    args = parser.parse_args()

    cfg = yaml.safe_load(Path(args.data).read_text())
    root = Path(cfg.get("path", Path(args.data).parent))
    split = "val" if args.split == "valid" else args.split
    img_dir = root / cfg[split]
    label_dir = Path(str(img_dir).replace("/images", "/labels"))
    names = {int(k): v for k, v in cfg["names"].items()}
    wanted = set(args.cls) if args.cls else None

    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    items = []
    for p in images:
        boxes = load_labels(label_dir / f"{p.stem}.txt")
        if wanted is not None:
            boxes = [b for b in boxes if b[0] in wanted]
            if not boxes:
                continue
        items.append((p, boxes))

    if not items:
        raise SystemExit(f"Aucune image trouvée dans {img_dir} (labels : {label_dir})")
    print(f"{len(items)} images, labels dans {label_dir}")

    i = 0
    while True:
        path, boxes = items[i]
        img = draw(cv2.imread(str(path)), boxes, names)
        cv2.imshow("labels", img)
        cv2.setWindowTitle("labels", f"[{i + 1}/{len(items)}] {path.name}")
        key = cv2.waitKey(0) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("p"):
            i = (i - 1) % len(items)
        else:
            i = (i + 1) % len(items)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
