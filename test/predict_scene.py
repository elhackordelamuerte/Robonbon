#!/usr/bin/env python3
"""Lance le modèle entraîné sur les photos de la scène et affiche les détections.

Usage :
    python test/predict_scene.py
    python test/predict_scene.py --weights runs/detect/train-6/weights/best.pt --conf 0.25 --source images

Pour chaque image : liste des détections dans le terminal, image annotée affichée
(n / espace = suivante, q / Échap = quitter) et sauvegardée dans runs/predict_scene/.
"""
import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default="runs/detect/train-6/weights/best.pt")
    parser.add_argument("--source", default="images")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    model = YOLO(args.weights)
    out_dir = Path("runs/predict_scene")
    out_dir.mkdir(parents=True, exist_ok=True)

    if not args.no_show:
        cv2.namedWindow("predict", cv2.WINDOW_NORMAL)

    sources = sorted(p for p in Path(args.source).iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    for path in sources:
        result = model.predict(str(path), conf=args.conf, save_txt=True, device=args.device, verbose=False)[0]
        boxes = result.boxes

        print(f"\n=== {path.name} : {len(boxes)} détection(s)")
        rows = []
        for cls, conf, xyxy in zip(boxes.cls.tolist(), boxes.conf.tolist(), boxes.xyxy.tolist()):
            x1, y1, x2, y2 = xyxy
            rows.append((result.names[int(cls)], conf, x1, y1, x2, y2))
        for name, conf, x1, y1, x2, y2 in sorted(rows, key=lambda r: r[0]):
            print(f"  {name:<13} {conf:.2f}  bbox=({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})  bas=({(x1 + x2) / 2:.0f}, {y2:.0f})")

        annotated = result.plot()
        cv2.imwrite(str(out_dir / path.name), annotated)

        if not args.no_show:
            cv2.imshow("predict", annotated)
            cv2.resizeWindow("predict", 800, 600)
            cv2.setWindowTitle("predict", path.name)
            if cv2.waitKey(0) & 0xFF in (ord("q"), 27):
                break

    cv2.destroyAllWindows()
    print(f"\nImages annotées dans {out_dir}/")


if __name__ == "__main__":
    main()
