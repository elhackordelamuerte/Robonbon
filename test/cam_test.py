#!/usr/bin/env python3
"""Test jetable : vérifie que la caméra s'ouvre et affiche le flux.

Usage : python3 cam_test.py [index]   (défaut 0)
Quitter : touche q
"""
import sys

import cv2

index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cap = cv2.VideoCapture(index)
if not cap.isOpened():
    sys.exit(f"Impossible d'ouvrir /dev/video{index}")

ok, frame = cap.read()
if not ok:
    sys.exit("Caméra ouverte mais aucune image lue")
print(f"OK /dev/video{index} : {frame.shape[1]}x{frame.shape[0]}, {cap.get(cv2.CAP_PROP_FPS):.0f} fps")

while True:
    ok, frame = cap.read()
    if not ok:
        print("Lecture échouée")
        break
    cv2.imshow(f"video{index}", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
