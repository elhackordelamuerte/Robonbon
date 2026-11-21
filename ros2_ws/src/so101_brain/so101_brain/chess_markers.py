#!/usr/bin/env python3
"""chess_markers.py — affiche l'échiquier dans RViz (MarkerArray sur /chess/markers, repère world).

- /chess/state     (perception) : plateau + pièces vues par la caméra, blanches / noires
- /chess/predicted (world_model) : pièces après les coups prédits, en vert (rouge si tombée),
                                   et le trajet du bout de pince

Position du plateau : celle trouvée par le marqueur ArUco (dans /chess/state) si présente,
sinon paramètres board_x, board_y (m), board_yaw (°), par défaut BOARD_X/Y/YAW de docker/.env.
"""
import json
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from so101_brain.world_model import Board


def _color(m, r, g, b, a=1.0):
    m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), float(a)


class ChessMarkers(Node):
    def __init__(self):
        super().__init__("chess_markers")
        self.declare_parameter("board_x", float(os.environ.get("BOARD_X", 0.10)))
        self.declare_parameter("board_y", float(os.environ.get("BOARD_Y", 0.10)))
        self.declare_parameter("board_yaw", float(os.environ.get("BOARD_YAW", -90.0)))
        self._state = None
        self._pred = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "chess/state", self._cb_state, qos)
        self.create_subscription(String, "chess/predicted", self._cb_pred, qos)
        self._pub = self.create_publisher(MarkerArray, "chess/markers", 10)
        self.create_timer(1.0, self._publish)

    def _board(self, square):
        return Board((self.get_parameter("board_x").value, self.get_parameter("board_y").value),
                     self.get_parameter("board_yaw").value, square)

    def _cb_state(self, msg):
        self._state = json.loads(msg.data)

    def _cb_pred(self, msg):
        self._pred = json.loads(msg.data)

    def _marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _board_markers(self, board: Board):
        s = board.square
        paper = self._marker("plateau", 0, Marker.CUBE)
        c = board.to_robot((4 * s, 4 * s))
        paper.pose.position.x, paper.pose.position.y, paper.pose.position.z = float(c[0]), float(c[1]), 0.0005
        yaw = np.arctan2(board.R[1, 0], board.R[0, 0])
        paper.pose.orientation.z, paper.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
        paper.scale.x = paper.scale.y = 8 * s + 0.02
        paper.scale.z = 0.001
        _color(paper, 0.95, 0.95, 0.95)
        dark = self._marker("plateau", 1, Marker.CUBE_LIST)
        dark.scale.x = dark.scale.y = s * 0.98
        dark.scale.z = 0.0012
        dark.pose.orientation = paper.pose.orientation
        _color(dark, 0.08, 0.08, 0.08)
        # CUBE_LIST : points dans le repère du marqueur (tourné de yaw) -> on tourne à l'envers
        Rinv = board.R.T
        for f in range(8):
            for r in range(8):
                if (f + r) % 2 == 0:  # a1 foncée
                    p = Rinv @ (board.to_robot(((f + 0.5) * s, (r + 0.5) * s)))
                    dark.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0007))
        return [paper, dark]

    def _piece_markers(self, ns, pieces, board: Board, color_fn, start_id=0):
        out = []
        for i, p in enumerate(pieces):
            if p.get("captured") or p.get("square") is None:
                continue
            m = self._marker(ns, start_id + i, Marker.CYLINDER)
            xy = board.to_robot((p["x"], p["y"]))
            m.scale.x = m.scale.y = 2 * p["base_r"]
            m.scale.z = p["height"]
            m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
            if p.get("fallen"):  # couchée : cylindre tourné de 90° autour de y
                m.pose.position.z = p["base_r"]
                m.pose.orientation.y = m.pose.orientation.w = float(np.sqrt(0.5))
            else:
                m.pose.position.z = p["height"] / 2
            _color(m, *color_fn(p))
            out.append(m)
        return out

    def _publish(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if self._state is not None:
            board = self._board(self._state.get("square_size", 0.025))
            pose = self._state.get("board_pose")
            if pose:  # position trouvée par le marqueur ArUco
                board = Board((pose["x"], pose["y"]), pose["yaw"], self._state.get("square_size", 0.025))
            arr.markers += self._board_markers(board)
            arr.markers += self._piece_markers(
                "pieces", self._state["pieces"], board,
                lambda p: (0.9, 0.85, 0.7) if p["color"] == "white" else (0.15, 0.15, 0.15))
        if self._pred is not None:
            b = self._pred.get("board", {})
            board = Board((b.get("x", 0.10), b.get("y", 0.10)), b.get("yaw", -90.0), self._pred.get("square_size", 0.025))
            arr.markers += self._piece_markers(
                "prediction", self._pred["pieces"], board,
                lambda p: (1.0, 0.1, 0.1, 0.8) if p.get("fallen") else (0.2, 0.9, 0.3, 0.5))
            if self._pred.get("path"):
                line = self._marker("trajet pince", 0, Marker.LINE_STRIP)
                line.scale.x = 0.002
                _color(line, 0.2, 0.6, 1.0)
                line.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in self._pred["path"]]
                arr.markers.append(line)
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ChessMarkers()
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
