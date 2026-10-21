# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-python==4.10.0.84", "numpy<2.3", "pillow==11.0.0"]
# ///
"""Génère les cibles de calibration à imprimer (PDF A4 + PNG 300 DPI) dans calib_targets/.

    uv run scripts/generate_targets.py

  - damier_9x6_25mm.pdf : damier 10x7 cases de 25 mm (9x6 coins intérieurs) -> calibrate_intrinsics.py
  - aruco_4x4_id0_80mm.pdf : marqueur ArUco DICT_4X4_50, id 0, 80 mm de côté -> calibrate_pose.py

Imprimer à 100 % (« taille réelle », sans « ajuster à la page »), puis MESURER au réglet :
la taille réelle d'une case / du marqueur se passe aux scripts (--case-mm, --marqueur-mm).
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

DPI = 300
MM_PAR_POUCE = 25.4
A4_MM = (210, 297)
DOSSIER = Path(__file__).resolve().parent.parent / "calib_targets"

COINS_DAMIER = (9, 6)  # coins intérieurs (colonnes, lignes)
CASE_MM = 25.0
MARQUEUR_MM = 80.0
MARQUEUR_ID = 0
DICTIONNAIRE = cv2.aruco.DICT_4X4_50


def mm_vers_px(mm: float) -> int:
    return round(mm / MM_PAR_POUCE * DPI)


def page_a4() -> np.ndarray:
    return np.full((mm_vers_px(A4_MM[1]), mm_vers_px(A4_MM[0])), 255, np.uint8)


def legende(page: np.ndarray, texte: str) -> None:
    cv2.putText(page, texte, (mm_vers_px(15), mm_vers_px(15)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 3)


def damier() -> np.ndarray:
    page = page_a4()
    case = mm_vers_px(CASE_MM)
    colonnes, lignes = COINS_DAMIER[0] + 1, COINS_DAMIER[1] + 1
    x0 = (page.shape[1] - colonnes * case) // 2
    y0 = (page.shape[0] - lignes * case) // 2
    for i in range(lignes):
        for j in range(colonnes):
            if (i + j) % 2 == 0:
                page[y0 + i * case: y0 + (i + 1) * case, x0 + j * case: x0 + (j + 1) * case] = 0
    legende(page, f"Damier {COINS_DAMIER[0]}x{COINS_DAMIER[1]} coins, case {CASE_MM:.0f} mm - imprimer a 100 %")
    return page


def marqueur() -> np.ndarray:
    page = page_a4()
    cote = mm_vers_px(MARQUEUR_MM)
    image = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(DICTIONNAIRE), MARQUEUR_ID, cote)
    x0, y0 = (page.shape[1] - cote) // 2, (page.shape[0] - cote) // 2
    page[y0: y0 + cote, x0: x0 + cote] = image
    # Flèche = axe +x du marqueur (à orienter vers l'avant du robot, cf. docs/JUMEAU_NUMERIQUE.md).
    milieu_y = y0 + cote // 2
    cv2.arrowedLine(page, (x0 + cote + mm_vers_px(5), milieu_y), (x0 + cote + mm_vers_px(30), milieu_y), 0, 6,
                    tipLength=0.3)
    cv2.putText(page, "+x", (x0 + cote + mm_vers_px(12), milieu_y - mm_vers_px(5)), cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 3)
    legende(page, f"ArUco 4x4_50 id {MARQUEUR_ID}, {MARQUEUR_MM:.0f} mm - imprimer a 100 %")
    return page

def main() -> None:
    DOSSIER.mkdir(exist_ok=True)
    for nom, image in (("damier_9x6_25mm", damier()), ("aruco_4x4_id0_80mm", marqueur())):
        page = Image.fromarray(image)
        # Le DPI fixe la taille physique : le PDF fait exactement 210 x 297 mm.
        page.save(DOSSIER / f"{nom}.pdf", resolution=DPI)
        page.save(DOSSIER / f"{nom}.png", dpi=(DPI, DPI))
        print(f"✅ calib_targets/{nom}.pdf (+ .png)")


if __name__ == "__main__":
    main()
