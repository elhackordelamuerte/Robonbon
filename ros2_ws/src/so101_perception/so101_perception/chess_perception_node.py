#!/usr/bin/env python3
"""chess_perception_node.py — état de l'échiquier par la caméra (conteneur perception).

Toutes les `period` secondes : image -> YOLO -> pièces. L'état publié sur /chess/state est le
vote des `frames` dernières images : une case n'est gardée que si une pièce y est vue dans au
moins 60 % d'entre elles (élimine la main, les fausses détections, les doublons).
Le plateau ne bouge pas : sa position est la médiane des dernières détections réussies
(une détection ratée ou fausse ne change rien).

Position du plateau par rapport au bras : automatique si le marqueur ArUco (id marker_id, côté
marker_size) est vu. Il est scotché sur la table à un endroit fixe : centre (marker_x, marker_y)
dans le repère du bras, haut du marqueur orienté à marker_yaw (0 = vers l'avant du bras).
La pose est publiée dans l'état ("board_pose" : coin a1 et orientation), et a1 est choisi
automatiquement : le coin foncé le plus proche du bras.

/chess/state : std_msgs/String, JSON {"square_size": s, "pieces": [{id, color, type, square,
x, y, tilt_deg, fallen, height, base_r, ...}]}, x/y dans le repère plateau (m, origine au coin
extérieur de a1). QoS transient local : un nœud lancé plus tard reçoit le dernier état.
Le dernier état et l'image annotée sont aussi écrits dans `out_dir` (latest.json, latest.jpg).

Paramètres :
    source      /dev/videoN (caméra, ex. DroidCam) ou chemin d'une photo
    weights     poids YOLO
    frames      nombre d'images votées
    conf, square_size, fx (focale / largeur image), flip (a1 <-> h8), period, out_dir
"""
import json
import os
from collections import deque
from pathlib import Path

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

import numpy as np

from so101_perception.chess_vision import (SQUARE_SIZE, board_from_corners, board_pose_from_marker,
                                           detect_board, detections_from_yolo, find_marker, read_state,
                                           vote)

PAN_AXIS = np.array([0.0388, 0.0])  # axe de rotation de la base dans le repère du bras


class ChessPerceptionNode(Node):
    def __init__(self):
        super().__init__("chess_perception")
        ANY = ParameterDescriptor(dynamic_typing=True)  # accepte 0 comme 0.0 depuis docker/.env
        self.declare_parameter("source", "/dev/video0")
        self.declare_parameter("weights", "/runs/detect/train-17/weights/best.pt")
        self.declare_parameter("conf", 0.25, ANY)
        self.declare_parameter("frames", 5)
        self.declare_parameter("square_size", SQUARE_SIZE, ANY)
        self.declare_parameter("fx", 0.72, ANY)
        self.declare_parameter("flip", False)
        self.declare_parameter("period", 2.0, ANY)
        self.declare_parameter("out_dir", "/runs/scene")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.08, ANY)
        self.declare_parameter("marker_x", 0.10, ANY)
        self.declare_parameter("marker_y", 0.0, ANY)
        self.declare_parameter("marker_yaw", 0.0, ANY)

        from ultralytics import YOLO  # import lourd : seulement ici

        self._model = YOLO(self.get_parameter("weights").value)
        self._source = self.get_parameter("source").value
        self._is_camera = self._source.startswith("/dev/video")
        self._cap = None
        self._corners = deque(maxlen=7)  # coins du plateau des dernières détections réussies
        self._poses = deque(maxlen=7)    # poses plateau -> bras des dernières images avec le marqueur
        self._flip = False
        self._history = deque(maxlen=self.get_parameter("frames").value)
        self._out = Path(self.get_parameter("out_dir").value)
        self._out.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(String, "chess/state", qos)
        self.create_timer(float(self.get_parameter("period").value), self._step)
        self.get_logger().info(f"Chess perception prête (source {self._source})")

    def _grab(self):
        if not self._is_camera:
            return cv2.imread(self._source)
        # (ré)ouverture à chaque échec : la caméra peut apparaître après le démarrage du nœud
        if self._cap is None or not self._cap.isOpened():
            # V4L2 n'ouvre pas par chemin dans cette version d'OpenCV : on passe le numéro
            self._cap = cv2.VideoCapture(int(self._source.removeprefix("/dev/video")), cv2.CAP_V4L2)
        for _ in range(5):  # vide le tampon de la caméra pour avoir une image récente
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok:
            self._cap.release()
            self._cap = None
            self.get_logger().warn(f"caméra : {self._why()}", throttle_duration_sec=10.0)
            return None
        return frame

    def _why(self):
        """Raison système de l'échec d'ouverture de la caméra (droits, périphérique occupé...)."""
        try:
            fd = os.open(self._source, os.O_RDWR | os.O_NONBLOCK)
            os.close(fd)
            return f"{self._source} s'ouvre, mais OpenCV ne lit pas d'image (format, pas de flux DroidCam ?)"
        except OSError as e:
            return f"{self._source} : {e.strerror} (errno {e.errno})"

    def _step(self):
        img = self._grab()
        if img is None:
            self.get_logger().warn(f"pas d'image depuis {self._source}", throttle_duration_sec=5.0)
            return
        dets = detections_from_yolo(self._model, img, float(self.get_parameter("conf").value))
        square = float(self.get_parameter("square_size").value)
        marker = find_marker(img, self.get_parameter("marker_id").value)
        try:
            c = detect_board(img, square, hide=marker).corners_px
            if self._corners:
                # le plateau est symétrique : garde l'orientation (a1) des détections précédentes
                ref = np.median(np.array(self._corners), axis=0)
                c = min((c, np.roll(c, 2, axis=0)), key=lambda k: np.abs(k - ref).sum())
            elif self.get_parameter("flip").value:
                c = np.roll(c, 2, axis=0)
            self._corners.append(c)
        except RuntimeError as e:
            if not self._corners:
                self.get_logger().warn(f"plateau jamais trouvé : {e}", throttle_duration_sec=5.0)
                return
        board = board_from_corners(np.median(np.array(self._corners), axis=0), square)
        if marker is not None:
            self._update_pose(board, marker)
        if self._flip:
            board = board.flipped()
        state, annotated, _ = read_state(img, dets, square_size=square, fx_ratio=float(self.get_parameter("fx").value), board=board)
        if marker is not None:
            cv2.polylines(annotated, [marker.astype(np.int32)], True, (0, 255, 255), 3)
        if self._poses:
            x, y, yaw = np.median(np.array(self._poses), axis=0)
            state["board_pose"] = dict(x=round(float(x), 4), y=round(float(y), 4), yaw=round(float(yaw), 2))
        cv2.imwrite(str(self._out / "latest.jpg"), annotated)

        self._history.append(state["pieces"])
        if len(self._history) < self._history.maxlen:
            return
        state["pieces"] = vote(list(self._history))
        self._pub.publish(String(data=json.dumps(state)))
        (self._out / "latest.json").write_text(json.dumps(state, indent=1))
        squares = " ".join(f"{p['type']} {p['square']}" for p in state["pieces"])
        if squares != getattr(self, "_last_log", None):  # n'affiche que les changements
            self._last_log = squares
            self.get_logger().info(f"{len(state['pieces'])} pièces : {squares or '-'}")

    def _update_pose(self, board, marker):
        """Pose du plateau dans le repère du bras grâce au marqueur ; a1 = coin foncé le plus
        proche du bras (sinon on retourne le plateau de 180°)."""
        args = [float(self.get_parameter(n).value) for n in ("marker_size", "marker_x", "marker_y", "marker_yaw")]
        pose = board_pose_from_marker(board, marker, *args)
        a = np.radians(pose["yaw"])
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        a1 = np.array([pose["x"], pose["y"]])
        h8 = a1 + R @ np.full(2, 8 * board.square_size)
        flip = np.linalg.norm(h8 - PAN_AXIS) < np.linalg.norm(a1 - PAN_AXIS)
        if flip:
            pose = board_pose_from_marker(board.flipped(), marker, *args)
        if self._flip != flip:
            self._poses.clear()  # orientation changée : on repart de zéro
            self._history.clear()
        self._flip = flip
        self._poses.append((pose["x"], pose["y"], pose["yaw"]))

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChessPerceptionNode()
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
