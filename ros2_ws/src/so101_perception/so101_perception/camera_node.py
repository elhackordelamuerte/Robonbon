#!/usr/bin/env python3
"""camera_node.py — caméra réelle -> topics ROS (/camera/image_raw, /camera/camera_info).

Lit une source OpenCV : URL du flux MJPEG de l'hôte (scripts/camera_stream.py, cas macOS)
ou index de périphérique (/dev/videoN sous Linux). Publie sur les mêmes types de messages
que la caméra simulée, pour que perception_node fonctionne sans changement.

Paramètres :
  source            URL ou index ("0"), défaut : flux de l'hôte via host.docker.internal
  fps               cadence de publication (défaut 10)
  calibration_file  YAML produit par cv2 (camera_matrix, dist_coeffs) ; sinon K approximée
  hfov_deg          champ horizontal supposé quand il n'y a pas de calibration (défaut 70°)
  frame_id          repère de l'image (défaut "camera")
"""
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image

SOURCE_DEFAUT = "http://host.docker.internal:8081/stream.mjpg"
DELAI_RECONNEXION_S = 1.0


def ouvrir_source(source: str) -> cv2.VideoCapture:
    return cv2.VideoCapture(int(source)) if source.isdigit() else cv2.VideoCapture(source)


def lire_calibration(chemin: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Lit camera_matrix et dist_coeffs d'un YAML OpenCV (cv2.FileStorage)."""
    if not chemin or not Path(chemin).is_file():
        return None
    fichier = cv2.FileStorage(chemin, cv2.FILE_STORAGE_READ)
    K, dist = fichier.getNode("camera_matrix").mat(), fichier.getNode("dist_coeffs").mat()
    fichier.release()
    return (K, dist.flatten()) if K is not None else None


class CameraNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("so101_camera")
        self._source = self.declare_parameter("source", SOURCE_DEFAUT).value
        fps = float(self.declare_parameter("fps", 10.0).value)
        self._frame_id = self.declare_parameter("frame_id", "camera").value
        self._hfov = math.radians(float(self.declare_parameter("hfov_deg", 70.0).value))
        self._calibration = lire_calibration(self.declare_parameter("calibration_file", "").value)
        if self._calibration is None:
            self.get_logger().warn(
                "no calibration file: K approximated from hfov_deg (fine for 2D detection, "
                "NOT for accurate 3D — run a checkerboard calibration)")

        self._pub_image = self.create_publisher(Image, "/camera/image_raw", 1)
        self._pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", 1)
        self._image: np.ndarray | None = None
        self._verrou = threading.Lock()
        threading.Thread(target=self._boucle_capture, daemon=True).start()
        self.create_timer(1.0 / fps, self._publier)
        self.get_logger().info(f"Camera node ready, source: {self._source}")

    def _boucle_capture(self):
        """Lit en continu (vide le tampon du flux) et garde la dernière image."""
        capture = ouvrir_source(self._source)
        while rclpy.ok():
            ok, image = capture.read()
            if not ok:
                self.get_logger().warn(f"cannot read {self._source}, retrying…", throttle_duration_sec=5.0)
                capture.release()
                time.sleep(DELAI_RECONNEXION_S)
                capture = ouvrir_source(self._source)
                continue
            with self._verrou:
                self._image = image

    def _camera_info(self, largeur: int, hauteur: int) -> CameraInfo:
        if self._calibration is not None:
            K, dist = self._calibration
        else:
            f = largeur / (2.0 * math.tan(self._hfov / 2.0))
            K, dist = np.array([[f, 0, largeur / 2], [0, f, hauteur / 2], [0, 0, 1]]), np.zeros(5)
        info = CameraInfo(width=largeur, height=hauteur, distortion_model="plumb_bob")
        info.k = K.flatten().astype(float).tolist()
        info.d = dist.astype(float).tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publier(self):
        with self._verrou:
            image = self._image
        if image is None:
            return
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        hauteur, largeur = rgb.shape[:2]
        msg = Image(height=hauteur, width=largeur, encoding="rgb8", step=largeur * 3, data=rgb.tobytes())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        info = self._camera_info(largeur, hauteur)
        info.header = msg.header
        self._pub_image.publish(msg)
        self._pub_info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
