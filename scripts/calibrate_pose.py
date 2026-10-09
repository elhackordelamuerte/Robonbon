# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-python==4.10.0.84", "numpy<2.3", "pyyaml==6.0.2"]
# ///
"""Pose de la caméra dans le repère du robot (cam_T) grâce au marqueur ArUco posé sur la table.

Prérequis : scripts/camera_stream.py lancé, ros2_ws/calib/camera.yaml (calibrate_intrinsics.py),
marqueur calib_targets/aruco_4x4_id0_80mm.pdf imprimé à 100 %, posé À PLAT sur la table :

    centre du marqueur à (--x, --y) mètres du centre de la base du robot (repère base_link :
    x vers l'avant du robot, y vers sa gauche), flèche « +x » du marqueur vers l'avant du robot.

    uv run scripts/calibrate_pose.py --x 0.25 --y 0.0 --marqueur-mm 80

Le robot n'a pas besoin de bouger. La caméra NE DOIT PLUS BOUGER ensuite (sinon recommencer).
Résultat : ros2_ws/calib/camera_pose.yaml (cam_T 4x4, caméra -> robot, convention OpenCV),
lu par perception_node en mode caméra réelle.
"""
import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parent.parent
FICHIER_K = RACINE / "ros2_ws" / "calib" / "camera.yaml"
SORTIE = RACINE / "ros2_ws" / "calib" / "camera_pose.yaml"
SOURCE_DEFAUT = "http://localhost:8081/stream.mjpg"
MARQUEUR_ID = 0
NB_MESURES = 30
DISPERSION_MAX_MM = 3.0


def lire_intrinseques() -> tuple[np.ndarray, np.ndarray]:
    if not FICHIER_K.is_file():
        sys.exit(f"❌ {FICHIER_K.relative_to(RACINE)} absent : lancer d'abord scripts/calibrate_intrinsics.py")
    fichier = cv2.FileStorage(str(FICHIER_K), cv2.FILE_STORAGE_READ)
    K, dist = fichier.getNode("camera_matrix").mat(), fichier.getNode("dist_coeffs").mat()
    fichier.release()
    return K, dist


def matrice(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rotation, translation.ravel()
    return T


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=SOURCE_DEFAUT)
    parser.add_argument("--x", type=float, required=True, help="centre du marqueur, vers l'avant du robot (m)")
    parser.add_argument("--y", type=float, required=True, help="centre du marqueur, vers la gauche du robot (m)")
    parser.add_argument("--yaw-deg", type=float, default=0.0, help="rotation de la flèche +x par rapport à l'avant du robot")
    parser.add_argument("--marqueur-mm", type=float, default=80.0, help="côté RÉEL du marqueur imprimé (mm)")
    args = parser.parse_args()

    K, dist = lire_intrinseques()
    demi = args.marqueur_mm / 2000.0
    # Coins du marqueur dans son repère (x droite, y haut de la page, z sortant de la feuille),
    # dans l'ordre renvoyé par OpenCV : haut-gauche, haut-droite, bas-droite, bas-gauche.
    coins_marqueur = np.array([[-demi, demi, 0], [demi, demi, 0], [demi, -demi, 0], [-demi, -demi, 0]], np.float32)
    detecteur = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

    capture = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    if not capture.isOpened():
        sys.exit(f"❌ Source {args.source} illisible : scripts/camera_stream.py est-il lancé ?")
    poses, image_finale = [], None
    print(f"🔎 Recherche du marqueur ArUco id {MARQUEUR_ID} ({NB_MESURES} mesures)…")
    essais = 0
    while len(poses) < NB_MESURES and essais < NB_MESURES * 20:
        essais += 1
        ok, image = capture.read()
        if not ok:
            continue
        coins, ids, _ = detecteur.detectMarkers(image)
        if ids is None or MARQUEUR_ID not in ids.ravel():
            continue
        c = coins[list(ids.ravel()).index(MARQUEUR_ID)].reshape(4, 2)
        ok, rvec, tvec = cv2.solvePnP(coins_marqueur, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            poses.append((rvec, tvec))
            image_finale = image.copy()
            cv2.aruco.drawDetectedMarkers(image_finale, [c.reshape(1, 4, 2)], np.array([[MARQUEUR_ID]]))
            cv2.drawFrameAxes(image_finale, K, dist, rvec, tvec, demi * 2)
    capture.release()
    if len(poses) < NB_MESURES // 2:
        sys.exit("❌ Marqueur non détecté : vérifier qu'il est entièrement visible, à plat, bien éclairé")

    translations = np.array([t.ravel() for _, t in poses])
    dispersion_mm = translations.std(axis=0).max() * 1000
    rvec, tvec = poses[len(poses) // 2][0], np.median(translations, axis=0)
    T_cam_marqueur = matrice(cv2.Rodrigues(rvec)[0], tvec)  # marqueur -> caméra
    lacet = math.radians(args.yaw_deg)
    R_robot_marqueur = np.array([[math.cos(lacet), -math.sin(lacet), 0], [math.sin(lacet), math.cos(lacet), 0], [0, 0, 1]])
    T_robot_marqueur = matrice(R_robot_marqueur, np.array([args.x, args.y, 0.0]))  # marqueur à plat sur la table
    T_robot_cam = T_robot_marqueur @ np.linalg.inv(T_cam_marqueur)

    SORTIE.parent.mkdir(parents=True, exist_ok=True)
    # Conserve la correction empirique de la position du cube (mesurée au grab, cf. perception_node).
    correction = (yaml.safe_load(SORTIE.read_text()) or {}).get("correction_objet") if SORTIE.is_file() else None
    contenu = {
        "cam_T": [round(float(v), 6) for v in T_robot_cam.ravel()],
        "convention": "opencv",
        "marqueur": {"id": MARQUEUR_ID, "cote_mm": args.marqueur_mm, "x": args.x, "y": args.y, "yaw_deg": args.yaw_deg},
        "dispersion_mm": round(float(dispersion_mm), 2),
    }
    if correction is not None:
        contenu["correction_objet"] = correction
    SORTIE.write_text(yaml.safe_dump(contenu, sort_keys=False))
    cv2.imwrite(str(SORTIE.with_suffix(".jpg")), image_finale)
    position = T_robot_cam[:3, 3]
    axe_optique = T_robot_cam[:3, 2]
    print(f"\n✅ {SORTIE.relative_to(RACINE)} (+ image de contrôle {SORTIE.with_suffix('.jpg').name})")
    print(f"   caméra dans le repère robot : x={position[0]:.3f} y={position[1]:.3f} z={position[2]:.3f} m")
    print(f"   regarde vers le bas de {math.degrees(math.asin(-axe_optique[2])):.0f}° ; distance au marqueur "
          f"{np.linalg.norm(tvec) * 100:.1f} cm")
    print(f"   stabilité sur {len(poses)} mesures : {dispersion_mm:.1f} mm "
          f"{'✅' if dispersion_mm < DISPERSION_MAX_MM else '⚠️ (caméra ou marqueur qui bouge ?)'}")
    if position[2] <= 0:
        print("   ⚠️ caméra sous la table : vérifier l'orientation du marqueur (flèche +x vers l'avant du robot)")


if __name__ == "__main__":
    main()
