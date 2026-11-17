#!/usr/bin/env python3
"""perception_node.py — détection du cube rouge et de la boîte, recalage 3D caméra -> monde.

  <camera_ns>/image_raw     (sub)  image RGB (sim : /external_cam, caméra réelle : /camera)
  <camera_ns>/camera_info   (sub)  matrice intrinsèque K
  /object_position    (pub)  PoseStamped, centre du cube + yaw, repère world (~10 Hz)
  /drop_box_position  (pub)  PoseStamped, centre de la boîte, repère world
  /perception/debug   (pub)  image annotée (contours, centres) pour RViz

Procédé (cf. docs/GRAB_CAMERA.md) :
  1. segmentation HSV (rouge : deux plages autour de 0°/180° ; boîte : marron)
  2. plus grand contour -> centre (u, v) par les moments
  3. rayon K^-1 [u, v, 1], convention MuJoCo -> monde avec la pose caméra T
  4. intersection avec un plan horizontal connu (hypothèse : objet posé sur la table)

La pose caméra T (extrinsèques) vient du paramètre `cam_T` de ce nœud (calibration,
16 valeurs, ligne par ligne) ou, à défaut, du paramètre `cam_T` du driver (vérité terrain
de la sim, autorisée pour démarrer). Sans T : mode 2D (détection + image annotée seulement).

Paramètres : camera_ns, cam_T, cam_T_file (YAML de scripts/calibrate_pose.py), cam_T_depuis_driver, convention_mujoco (True en sim ; False pour un cam_T issu
de cv2.solvePnP, déjà en convention OpenCV), hsv_rouge (12 entiers = 2 plages min/max),
detecter_boite, boite_fixe ([x, y] publié si detecter_boite est faux), detecter_bol, hsv_bol, z_plan_bol, aire_min_cube_px.
"""
import json
import math
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.node
import yaml
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

# -- Seuils HSV (OpenCV : H 0-180) ---------------------------------------------
# Cube rouge : le rouge chevauche 0°/180°. V >= 100 exclut le marron de la boîte (V ~ 73).
ROUGE_BAS = [((0, 120, 100), (12, 255, 255)), ((160, 120, 100), (180, 255, 255))]
# Boîte marron (rgba 0.35 0.20 0.12) : H ~ 10, V ~ 55-75.
MARRON = [((5, 80, 30), (20, 255, 95))]

# Hauteurs des plans d'intersection (m). Le centre visible du cube mélange le dessus
# et les faces latérales : 2,75 cm minimise l'erreur (~4 mm en moyenne, mesuré en sim).
Z_PLAN_CUBE = 0.0275
Z_PLAN_DESSUS_CUBE = 0.05
Z_PLAN_BOITE = 0.036
# Bol en bois (zone de dépôt réelle) : centre de l'ellipse du contour projeté à mi-hauteur du bol.
BOIS = [((10, 40, 70), (24, 170, 200))]
Z_PLAN_BOL = 0.03
AIRE_MIN_BOL_PX = 300
PORTEE_BOL_M = (0.08, 0.45)  # rejette le fond de la salle (projeté à plusieurs mètres)
LISSAGE_BOL = 0.3  # filtre exponentiel sur la position du bol
PORTEE_CUBE_M = (0.05, 0.45)  # cube : distance à la base du bras plausible
# Vérification de prise : cube « dans la pince » si son centre est à moins de SEUIL px du segment
# projeté [TCP, TCP - DESCENTE_CUBE_M en z] (le cube pend sous la pointe des mâchoires).
SEUIL_PRISE_PX = 35.0
DESCENTE_CUBE_M = 0.04
# … et aucun cube encore posé sur la table à moins de RAYON_TABLE_M de la verticale de la pince
# (évite le faux positif d'un cube resté au sol qui, en perspective, apparaît juste sous la pince levée).
RAYON_TABLE_M = 0.06
AIRE_MIN_CUBE_PX = 40
AIRE_MIN_BOITE_PX = 60
MARGE_BORD_PX = 2  # contour touchant le bord = objet coupé -> ignoré

# Convention MuJoCo (axe optique -Z, Y vers le haut) -> OpenCV (Z devant, Y vers le bas).
MUJOCO_DEPUIS_OPENCV = np.diag([1.0, -1.0, -1.0])
NOEUD_DRIVER = "/so101_driver"
PARAM_CAM_T = "cam_T"
PERIODE_RECHERCHE_T_S = 2.0
PAUSE_APRES_REFUS_T_S = 30.0  # driver sans cam_T (bras réel) : ne pas le solliciter en boucle
FRAME_MONDE = "world"
NOYAU = np.ones((5, 5), np.uint8)


def masque_hsv(hsv: np.ndarray, plages: list) -> np.ndarray:
    """Union des plages HSV, nettoyée par fermeture morphologique."""
    masque = np.zeros(hsv.shape[:2], np.uint8)
    for bas, haut in plages:
        masque |= cv2.inRange(hsv, np.array(bas, np.uint8), np.array(haut, np.uint8))
    return cv2.morphologyEx(masque, cv2.MORPH_CLOSE, NOYAU)


def plus_grand_contour(masque: np.ndarray, aire_min: float):
    contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    return contour if cv2.contourArea(contour) >= aire_min else None


def touche_bord(contour, largeur: int, hauteur: int) -> bool:
    x, y, w, h = cv2.boundingRect(contour)
    return (x <= MARGE_BORD_PX or y <= MARGE_BORD_PX
            or x + w >= largeur - MARGE_BORD_PX or y + h >= hauteur - MARGE_BORD_PX)


def centre(contour) -> tuple[float, float]:
    m = cv2.moments(contour)
    return m["m10"] / m["m00"], m["m01"] / m["m00"]


def image_vers_numpy(msg: Image) -> np.ndarray:
    """sensor_msgs/Image rgb8/bgr8 -> tableau RGB (sans cv_bridge)."""
    image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
    image = image.reshape(msg.height, msg.width, 3)
    return image[:, :, ::-1] if msg.encoding == "bgr8" else image


def numpy_vers_image(image: np.ndarray, entete) -> Image:
    msg = Image(header=entete, height=image.shape[0], width=image.shape[1], encoding="rgb8")
    msg.step = image.shape[1] * 3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


class PerceptionNode(rclpy.node.Node):
    """Détecte le cube rouge et la boîte, publie leurs positions 3D (repère world)."""

    def __init__(self):
        super().__init__("so101_perception")
        self._verrou = threading.Lock()
        self._K_inv: np.ndarray | None = None
        self._T: np.ndarray | None = None  # pose caméra -> monde (4x4)
        self._attente_T = False
        self._refus_T = -PAUSE_APRES_REFUS_T_S  # instant (s) du dernier refus de cam_T
        # La boîte est fixe : on garde l'observation la plus complète (plus grande aire visible),
        # le bras la masquant souvent en partie.
        self._boite: PoseStamped | None = None
        self._aire_boite = 0.0

        ns = self.declare_parameter("camera_ns", "/external_cam").value.rstrip("/")
        self._flip = MUJOCO_DEPUIS_OPENCV if self.declare_parameter("convention_mujoco", True).value else np.eye(3)
        hsv = list(self.declare_parameter("hsv_rouge", [v for plage in ROUGE_BAS for b in plage for v in b]).value)
        self._plages_rouge = [(tuple(hsv[i:i + 3]), tuple(hsv[i + 3:i + 6])) for i in range(0, len(hsv), 6)]
        self._detecter_boite = self.declare_parameter("detecter_boite", True).value
        # Sans détection (caméra réelle) : position fixe de la zone de dépôt, en m dans le repère robot.
        boite_fixe = list(self.declare_parameter("boite_fixe", [0.05, -0.25]).value)
        # Caméra réelle : zone de dépôt = bol en bois détecté (sinon boite_fixe en attendant).
        self._detecter_bol = self.declare_parameter("detecter_bol", False).value
        hsv_bol = list(self.declare_parameter("hsv_bol", [v for plage in BOIS for b in plage for v in b]).value)
        self._plages_bol = [(tuple(hsv_bol[i:i + 3]), tuple(hsv_bol[i + 3:i + 6])) for i in range(0, len(hsv_bol), 6)]
        self._z_bol = float(self.declare_parameter("z_plan_bol", Z_PLAN_BOL).value)
        self._bol_xy: np.ndarray | None = None
        if not self._detecter_boite and len(boite_fixe) == 2:
            self._boite = PoseStamped()
            self._boite.header.frame_id = FRAME_MONDE
            self._boite.pose.position.x, self._boite.pose.position.y = float(boite_fixe[0]), float(boite_fixe[1])
            self._boite.pose.orientation.w = 1.0
        self._aire_min_cube = float(self.declare_parameter("aire_min_cube_px", AIRE_MIN_CUBE_PX).value)
        # Sim uniquement : la pose de la caméra virtuelle ne vaut rien pour une caméra réelle.
        self._T_depuis_driver = self.declare_parameter("cam_T_depuis_driver", True).value
        cam_T = list(self.declare_parameter("cam_T", [0.0]).value)
        # Fichier produit par scripts/calibrate_pose.py (ArUco) : prioritaire s'il existe.
        fichier_T = Path(self.declare_parameter("cam_T_file", "").value or "/inexistant")
        # Correction empirique de la position du cube [dx, dy] (m), mesurée au grab réel : compense le
        # champ caméra approximatif et l'affaissement du bras. Dans le même fichier que la calibration.
        self._correction = np.zeros(2)
        if fichier_T.is_file():
            contenu = yaml.safe_load(fichier_T.read_text())
            cam_T = contenu["cam_T"]
            self._correction = np.array(contenu.get("correction_objet", [0.0, 0.0]), dtype=float)
            self.get_logger().info(f"Camera pose from {fichier_T} (ArUco calibration)")
        if len(cam_T) == 16:
            self._T = np.array(cam_T, dtype=float).reshape(4, 4)
            self.get_logger().info(f"Camera pose from parameter: position {np.round(self._T[:3, 3], 3).tolist()}")

        self.create_subscription(CameraInfo, f"{ns}/camera_info", self._cb_camera_info, 1)
        self.create_subscription(Image, f"{ns}/image_raw", self._cb_image, 1)
        self._pub_objet = self.create_publisher(PoseStamped, "/object_position", 10)
        self._pub_boite = self.create_publisher(PoseStamped, "/drop_box_position", 10)
        self._pub_debug = self.create_publisher(Image, "/perception/debug", 1)
        # Vérification de prise (appelée par le brain bras levé) : le cube est-il collé à la pince ?
        self._derniere_image: np.ndarray | None = None
        self._pince: np.ndarray | None = None  # position mesurée du TCP (repère robot)
        self.create_subscription(PoseStamped, "/end_effector_pose", self._cb_pince, 10)
        self.create_service(Trigger, "/perception/verifier_prise", self._srv_verifier_prise)
        self._client_param = self.create_client(GetParameters, f"{NOEUD_DRIVER}/get_parameters")
        self.create_timer(PERIODE_RECHERCHE_T_S, self._fetch_cam_params)

        self.get_logger().info(f"Perception node ready on {ns} (waiting for camera_info and cam_T).")

    # ------------------------------------------------------------ calibration

    def _cb_camera_info(self, msg: CameraInfo):
        if self._K_inv is None:
            self._K_inv = np.linalg.inv(np.array(msg.k, dtype=float).reshape(3, 3))
            self.get_logger().info("Intrinsics K received.")


    def _fetch_cam_params(self):
        """Demande cam_T au driver (asynchrone) tant qu'on ne l'a pas."""
        if not self._T_depuis_driver or self._T is not None or self._attente_T \
                or not self._client_param.service_is_ready() \
                or self.get_clock().now().nanoseconds * 1e-9 - self._refus_T < PAUSE_APRES_REFUS_T_S:
            return
        self._attente_T = True
        futur = self._client_param.call_async(GetParameters.Request(names=[PARAM_CAM_T]))
        futur.add_done_callback(self._cb_cam_params)

    def _cb_cam_params(self, futur):
        self._attente_T = False
        valeurs = futur.result().values if futur.result() else []
        if not valeurs or len(valeurs[0].double_array_value) != 16:
            self._refus_T = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().warn(f"{NOEUD_DRIVER} has no '{PARAM_CAM_T}' parameter yet", throttle_duration_sec=10.0)
            return
        with self._verrou:
            self._T = np.array(valeurs[0].double_array_value).reshape(4, 4)
        self.get_logger().info(f"Camera pose received: position {np.round(self._T[:3, 3], 3).tolist()}")

    # ------------------------------------------------------------ traitement

    def _cb_image(self, msg: Image):
        if self._K_inv is None:
            return
        image = image_vers_numpy(msg)
        self._derniere_image = image
        debug = image.copy()
        contour = self._detect_object_2d(image, debug)
        if self._T is None:  # mode 2D : pas de pose caméra, pas de position 3D
            if contour is not None:
                u, v = centre(contour)
                self.get_logger().info(f"cube seen at pixel ({u:.0f}, {v:.0f}) — 2D only, no cam_T",
                                       throttle_duration_sec=2.0)
        else:
            objet = None if contour is None else self._localiser_objet(contour)
            boite = self._detect_box(image, debug) if self._detecter_boite else None
            if self._detecter_bol:
                self._detect_bowl(image, debug)
            self._publish_result(msg.header.stamp, objet, boite)
        self._pub_debug.publish(numpy_vers_image(debug, msg.header))

    def _detect_object_2d(self, image: np.ndarray, debug: np.ndarray):
        """Contour du cube rouge dans l'image, ou None s'il est absent ou coupé par le bord."""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        contour = self._contour_cube(masque_hsv(hsv, self._plages_rouge))
        if contour is None:
            return None
        hauteur, largeur = image.shape[:2]
        epaisseur = max(1, largeur // 320)
        if touche_bord(contour, largeur, hauteur):
            cv2.drawContours(debug, [contour], -1, (255, 160, 0), epaisseur)
            self.get_logger().warn("cube cut by the image border: ignored", throttle_duration_sec=5.0)
            return None
        u, v = centre(contour)
        cv2.drawContours(debug, [contour], -1, (0, 255, 0), epaisseur)
        cv2.drawMarker(debug, (int(u), int(v)), (0, 255, 0), cv2.MARKER_CROSS, 8 * epaisseur, epaisseur)
        return contour

    def _contour_cube(self, masque: np.ndarray):
        """Plus grand contour rouge ; avec une pose caméra, seulement s'il est à portée du bras
        (évite un vêtement ou une chaise rouge au fond de la salle, projeté à plusieurs mètres)."""
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(contour) < self._aire_min_cube:
                return None
            if self._T is None:
                return contour
            u, v = centre(contour)
            if PORTEE_CUBE_M[0] < np.hypot(*self._project_to_3d(u, v, Z_PLAN_CUBE)[:2]) < PORTEE_CUBE_M[1]:
                return contour
        return None

    def _localiser_objet(self, contour):
        """Position 3D (plan de la table) et yaw du cube à partir de son contour."""
        u, v = centre(contour)
        position = self._project_to_3d(u, v, Z_PLAN_CUBE)
        # Yaw : contour projeté sur le plan du dessus, puis rectangle d'aire minimale.
        points = np.array([self._project_to_3d(p[0][0], p[0][1], Z_PLAN_DESSUS_CUBE)[:2] for p in contour],
                          dtype=np.float32)
        yaw = math.radians(cv2.minAreaRect(points)[2]) % (math.pi / 2)
        return position, yaw

    def _detect_box(self, image: np.ndarray, debug: np.ndarray):
        """Retourne (position xyz, aire px) de la boîte, ou None."""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        contour = plus_grand_contour(masque_hsv(hsv, MARRON), AIRE_MIN_BOITE_PX)
        if contour is None:
            return None
        points = np.array([self._project_to_3d(p[0][0], p[0][1], Z_PLAN_BOITE)[:2] for p in contour],
                          dtype=np.float32)
        (x, y), _, _ = cv2.minAreaRect(points)
        cv2.drawContours(debug, [contour], -1, (0, 160, 255), 1)
        return np.array([x, y, 0.0]), cv2.contourArea(contour)

    def _cb_pince(self, msg: PoseStamped):
        p = msg.pose.position
        self._pince = np.array([p.x, p.y, p.z])

    def _projeter(self, point_robot: np.ndarray) -> np.ndarray | None:
        """Pixel (u, v) d'un point 3D du repère robot, ou None s'il est derrière la caméra."""
        p_cam = self._flip @ (self._T[:3, :3].T @ (point_robot - self._T[:3, 3]))
        if p_cam[2] <= 0:
            return None
        uv = np.linalg.inv(self._K_inv) @ p_cam
        return uv[:2] / uv[2]

    def _srv_verifier_prise(self, request, response):
        """Bras levé : le cube rouge est-il collé à la pince dans l'image ? (succès = prise réussie)."""
        if self._derniere_image is None or self._T is None or self._pince is None:
            response.success = False
            response.message = json.dumps({"verdict": "indeterminé", "raison": "pas d'image, de pose caméra ou de pose pince"})
            return response
        a = self._projeter(self._pince)
        b = self._projeter(self._pince - np.array([0.0, 0.0, DESCENTE_CUBE_M]))
        hsv = cv2.cvtColor(self._derniere_image, cv2.COLOR_RGB2HSV)
        contours, _ = cv2.findContours(masque_hsv(hsv, self._plages_rouge), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        meilleure = None
        for contour in contours:
            if cv2.contourArea(contour) < self._aire_min_cube / 2 or a is None or b is None:
                continue  # cube vu par en dessous / partiellement masqué par les mâchoires : seuil réduit
            c = np.array(centre(contour))
            t = np.clip(np.dot(c - a, b - a) / max(np.dot(b - a, b - a), 1e-9), 0.0, 1.0)
            d = float(np.linalg.norm(c - (a + t * (b - a))))
            if meilleure is None or d < meilleure[0]:
                meilleure = (d, c)
        # Cube toujours posé sur la table sous la pince ? (projection sur le plan de la table)
        au_sol = self._contour_cube(masque_hsv(hsv, self._plages_rouge))
        ecart_sol = None
        if au_sol is not None:
            ecart_sol = float(np.linalg.norm(self._project_to_3d(*centre(au_sol), Z_PLAN_CUBE)[:2] - self._pince[:2]))
        if meilleure is None:
            verdict, details = False, {"raison": "aucun cube rouge visible"}
        else:
            verdict = meilleure[0] < SEUIL_PRISE_PX and (ecart_sol is None or ecart_sol > RAYON_TABLE_M)
            details = {"distance_px": round(meilleure[0], 1), "seuil_px": SEUIL_PRISE_PX,
                       "pince_px": [round(float(v)) for v in a], "cube_px": [round(float(v)) for v in meilleure[1]]}
            if ecart_sol is not None and ecart_sol <= RAYON_TABLE_M:
                details["raison"] = f"cube encore sur la table à {ecart_sol * 100:.1f} cm sous la pince"
        details = {"verdict": "réussie" if verdict else "ratée", "pince_m": [round(float(v), 3) for v in self._pince], **details}
        self.get_logger().info(f"vérification de prise : {details}")
        response.success, response.message = bool(verdict), json.dumps(details, ensure_ascii=False)
        return response

    def _detect_bowl(self, image: np.ndarray, debug: np.ndarray) -> None:
        """Bol en bois : plus grand contour « bois » à portée du bras -> zone de dépôt (lissée)."""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        masque = masque_hsv(hsv, self._plages_bol)
        masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(contour) < AIRE_MIN_BOL_PX or len(contour) < 5:
                break
            (u, v), _, _ = cv2.fitEllipse(contour)
            xy = self._project_to_3d(u, v, self._z_bol)[:2]
            if not PORTEE_BOL_M[0] < np.hypot(*xy) < PORTEE_BOL_M[1]:
                continue  # fond de la salle, peau, etc.
            self._bol_xy = xy if self._bol_xy is None else (1 - LISSAGE_BOL) * self._bol_xy + LISSAGE_BOL * xy
            cv2.drawContours(debug, [contour], -1, (255, 200, 0), max(1, image.shape[1] // 320))
            cv2.drawMarker(debug, (int(u), int(v)), (255, 200, 0), cv2.MARKER_DIAMOND, 10, 2)
            self._boite = PoseStamped()
            self._boite.header.frame_id = FRAME_MONDE
            self._boite.pose.position.x, self._boite.pose.position.y = map(float, self._bol_xy)
            self._boite.pose.orientation.w = 1.0
            return

    def _project_to_3d(self, u: float, v: float, z_plan: float) -> np.ndarray:
        """Intersection du rayon du pixel (u, v) avec le plan horizontal z = z_plan (monde)."""
        rayon_cam = self._flip @ (self._K_inv @ np.array([u, v, 1.0]))
        rayon = self._T[:3, :3] @ rayon_cam
        origine = self._T[:3, 3]
        if abs(rayon[2]) < 1e-9:
            raise ValueError("ray parallel to the plane")
        return origine + (z_plan - origine[2]) / rayon[2] * rayon

    def _publish_result(self, stamp, objet, boite):
        if objet is not None:
            (x, y, z), yaw = objet
            x, y = x + self._correction[0], y + self._correction[1]
            msg = PoseStamped()
            msg.header.stamp, msg.header.frame_id = stamp, FRAME_MONDE
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self._pub_objet.publish(msg)
        if boite is not None and boite[1] >= self._aire_boite:
            position, self._aire_boite = boite
            msg = PoseStamped()
            msg.header.frame_id = FRAME_MONDE
            msg.pose.position.x, msg.pose.position.y = float(position[0]), float(position[1])
            msg.pose.orientation.w = 1.0
            self._boite = msg
        if self._boite is not None:  # meilleure observation de la boîte (fixe)
            self._boite.header.stamp = stamp
            self._pub_boite.publish(self._boite)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
