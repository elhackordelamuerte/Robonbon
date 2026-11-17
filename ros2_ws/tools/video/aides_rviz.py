#!/usr/bin/env python3
"""Aides d'affichage RViz pour la vidéo de démo (scripts/video_sim.sh).

  - TF statique world -> external_cam, tiré de cam_T (pose caméra utilisée par la perception) :
    RViz montre ainsi le repère de la caméra dans l'arbre des TF ;
  - marqueur cube (/video/cube) à la position publiée par la perception (/object_position) :
    le cube apparaît dans RViz là où la caméra le voit, et disparaît quand elle ne le voit plus.

Usage (dans un conteneur ROS) :  python3 aides_rviz.py <fichier yaml contenant cam_T>
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker

REPERE_MONDE = "world"
REPERE_CAMERA = "external_cam"
COTE_CUBE_M = 0.05
COULEUR_CUBE = (0.9, 0.1, 0.15, 0.9)  # rouge, comme le vrai cube
DUREE_VIE_CUBE_S = 0.6                # le cube s'efface si la perception ne le voit plus


def transformee_camera(cam_t: list[float], horodatage) -> TransformStamped:
    """cam_T (4x4 ligne par ligne, caméra -> monde) en TF world -> external_cam."""
    matrice = np.array(cam_t, dtype=float).reshape(4, 4)
    x, y, z, w = Rotation.from_matrix(matrice[:3, :3]).as_quat()
    tf = TransformStamped()
    tf.header.stamp = horodatage
    tf.header.frame_id = REPERE_MONDE
    tf.child_frame_id = REPERE_CAMERA
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(
        float, matrice[:3, 3])
    tf.transform.rotation.x, tf.transform.rotation.y = float(x), float(y)
    tf.transform.rotation.z, tf.transform.rotation.w = float(z), float(w)
    return tf


class AidesRviz(Node):
    def __init__(self, cam_t: list[float]):
        super().__init__("aides_rviz_video")
        self._diffuseur_tf = StaticTransformBroadcaster(self)
        self._diffuseur_tf.sendTransform(transformee_camera(cam_t, self.get_clock().now().to_msg()))
        self._pub_cube = self.create_publisher(Marker, "/video/cube", 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_objet, 10)

    def _cb_objet(self, msg: PoseStamped) -> None:
        cube = Marker()
        cube.header = msg.header
        cube.ns, cube.id = "cube", 0
        cube.type, cube.action = Marker.CUBE, Marker.ADD
        cube.pose = msg.pose
        if not any((cube.pose.orientation.x, cube.pose.orientation.y,
                    cube.pose.orientation.z, cube.pose.orientation.w)):
            cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = COTE_CUBE_M
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = COULEUR_CUBE
        cube.lifetime = Duration(seconds=DUREE_VIE_CUBE_S).to_msg()
        self._pub_cube.publish(cube)


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage : aides_rviz.py <fichier yaml contenant cam_T>")
    cam_t = yaml.safe_load(Path(sys.argv[1]).read_text())["cam_T"]
    rclpy.init()
    noeud = AidesRviz(cam_t)
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
