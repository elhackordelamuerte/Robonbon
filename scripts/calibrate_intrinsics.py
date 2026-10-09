# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-python==4.10.0.84", "numpy<2.3"]
# ///
"""Calibration intrinsèque de la caméra (matrice K + distorsion) : marqueur ArUco (défaut) ou damier.

Prérequis : scripts/camera_stream.py lancé (flux de la UGREEN), cible imprimée à 100 %, bien à plat.

    uv run scripts/calibrate_intrinsics.py --marqueur-mm 80          # ArUco seul (calib_targets/aruco_4x4_id0_80mm.pdf)
    uv run scripts/calibrate_intrinsics.py --cible damier --case-mm 25

Une fenêtre montre l'image : présenter la cible sous ~25 angles/distances différents (TRÈS inclinée,
dans les coins, près, loin). Une vue est prise automatiquement quand la cible est vue et a bougé.
`q` pour terminer avant. Résultat : ros2_ws/calib/camera.yaml (lu par camera_node).

Avec l'ArUco (4 coins par vue au lieu de 54), le modèle est contraint pour rester fiable :
centre optique au milieu de l'image, pixels carrés, pas de distorsion tangentielle, k3 = 0.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parent.parent
SORTIE = RACINE / "ros2_ws" / "calib" / "camera.yaml"
SOURCE_DEFAUT = "http://localhost:8081/stream.mjpg"
COINS = (9, 6)  # coins intérieurs du damier généré
NB_VUES = 25
MARQUEUR_ID = 0
# ArUco : peu de points par vue -> modèle simplifié (cf. docstring).
DRAPEAUX_ARUCO = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
                  | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
DELAI_ENTRE_VUES_S = 1.0
DEPLACEMENT_MIN_PX = 40.0  # le damier doit avoir bougé pour une nouvelle vue
RMS_BON_PX = 1.0
CRITERE = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def points_damier(case_m: float) -> np.ndarray:
    """Coins du damier dans son propre repère (plan z = 0), en mètres."""
    grille = np.zeros((COINS[0] * COINS[1], 3), np.float32)
    grille[:, :2] = np.mgrid[0: COINS[0], 0: COINS[1]].T.reshape(-1, 2) * case_m
    return grille


def points_marqueur(cote_m: float) -> np.ndarray:
    """Coins du marqueur (haut-gauche, haut-droite, bas-droite, bas-gauche), plan z = 0, en mètres."""
    d = cote_m / 2
    return np.array([[-d, d, 0], [d, d, 0], [d, -d, 0], [-d, -d, 0]], np.float32)


def detecter_damier(gris: np.ndarray, affichage: np.ndarray):
    trouve, coins = cv2.findChessboardCorners(gris, COINS, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not trouve:
        return None
    coins = cv2.cornerSubPix(gris, coins, (11, 11), (-1, -1), CRITERE)
    cv2.drawChessboardCorners(affichage, COINS, coins, trouve)
    return coins


def detecter_marqueur(detecteur, gris: np.ndarray, affichage: np.ndarray):
    coins, ids, _ = detecteur.detectMarkers(gris)
    if ids is None or MARQUEUR_ID not in ids.ravel():
        return None
    c = coins[list(ids.ravel()).index(MARQUEUR_ID)]
    cv2.cornerSubPix(gris, c.reshape(-1, 1, 2), (5, 5), (-1, -1), CRITERE)
    cv2.aruco.drawDetectedMarkers(affichage, [c], np.array([[MARQUEUR_ID]]))
    return c.reshape(-1, 1, 2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=SOURCE_DEFAUT, help="URL du flux ou index de caméra")
    parser.add_argument("--cible", choices=["aruco", "damier"], default="aruco")
    parser.add_argument("--marqueur-mm", type=float, default=80.0, help="côté RÉEL du marqueur imprimé (mm)")
    parser.add_argument("--case-mm", type=float, default=25.0, help="côté RÉEL d'une case du damier (mm)")
    parser.add_argument("--vues", type=int, default=NB_VUES)
    args = parser.parse_args()

    capture = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    if not capture.isOpened():
        sys.exit(f"❌ Source {args.source} illisible : scripts/camera_stream.py est-il lancé ?")
    aruco = args.cible == "aruco"
    objet = points_marqueur(args.marqueur_mm / 1000.0) if aruco else points_damier(args.case_mm / 1000.0)
    detecteur = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    points_objet, points_image, taille = [], [], None
    dernier_centre, derniere_prise = None, 0.0
    print(f"📷 Présenter {'le marqueur' if aruco else 'le damier'} sous des angles variés ({args.vues} vues, q pour finir)")
    while len(points_image) < args.vues:
        ok, image = capture.read()
        if not ok:
            continue
        gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        taille = gris.shape[::-1]
        affichage = image.copy()
        coins = detecter_marqueur(detecteur, gris, affichage) if aruco else detecter_damier(gris, affichage)
        trouve = coins is not None
        if trouve:
            centre = coins.reshape(-1, 2).mean(axis=0)
            a_bouge = dernier_centre is None or np.linalg.norm(centre - dernier_centre) > DEPLACEMENT_MIN_PX
            if a_bouge and time.monotonic() - derniere_prise > DELAI_ENTRE_VUES_S:
                points_objet.append(objet)
                points_image.append(coins)
                dernier_centre, derniere_prise = centre, time.monotonic()
                print(f"  ✅ vue {len(points_image)}/{args.vues}")
        cv2.putText(affichage, f"vues {len(points_image)}/{args.vues} - cible {'OK' if trouve else 'non vue'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if trouve else (0, 0, 255), 2)
        cv2.imshow("Calibration intrinseque (q pour finir)", affichage)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    capture.release()

    if len(points_image) < 8:
        sys.exit(f"❌ Seulement {len(points_image)} vues : il en faut au moins 8")
    if aruco:
        # Estimation initiale : focale ≈ largeur d'image (champ ~ 53°), centre au milieu.
        K0 = np.array([[taille[0], 0, taille[0] / 2], [0, taille[0], taille[1] / 2], [0, 0, 1]], float)
        rms, K, dist, _, _ = cv2.calibrateCamera(points_objet, points_image, taille, K0, np.zeros(5), flags=DRAPEAUX_ARUCO)
    else:
        rms, K, dist, _, _ = cv2.calibrateCamera(points_objet, points_image, taille, None, None)
    SORTIE.parent.mkdir(parents=True, exist_ok=True)
    fichier = cv2.FileStorage(str(SORTIE), cv2.FILE_STORAGE_WRITE)
    fichier.write("image_width", taille[0])
    fichier.write("image_height", taille[1])
    fichier.write("camera_matrix", K)
    fichier.write("dist_coeffs", dist)
    fichier.write("rms_px", rms)
    fichier.release()
    print(f"\n✅ {SORTIE.relative_to(RACINE)} ({taille[0]}x{taille[1]}, {len(points_image)} vues)")
    print(f"   fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}"
          f"  (champ horizontal ≈ {2 * np.degrees(np.arctan(taille[0] / (2 * K[0, 0]))):.0f}°)")
    print(f"   erreur de reprojection : {rms:.2f} px {'✅' if rms < RMS_BON_PX else '⚠️ (> 1 px : refaire avec plus de vues variées)'}")


if __name__ == "__main__":
    main()
