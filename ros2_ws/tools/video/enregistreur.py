#!/usr/bin/env python3
"""Enregistreur pour la vidéo de démo (scripts/video_sim.sh).

Pendant le tournage, note avec l'heure murale (time.time(), même horloge que la capture
ffmpeg de RViz) :
  - les logs du cerveau et de la perception (/rosout) -> evenements.jsonl
  - la position du cube publiée par la perception (/object_position), 5 Hz max
  - les images annotées de la perception (/perception/debug) -> camera/<t>.jpg, 10 Hz max
  - la vue d'ensemble MuJoCo du driver (/scene_cam/image_raw) -> scene/<t>.jpg, 10 Hz max

Usage (dans un conteneur ROS) :  python3 enregistreur.py <dossier_sortie>
Arrêt : SIGINT (Ctrl+C ou pkill -INT -f enregistreur.py).
"""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

NOEUDS_SUIVIS = ("so101_brain", "so101_perception")
PERIODE_OBJET_S = 0.2
PERIODE_IMAGE_S = 0.1
QUALITE_JPEG = 90


class Enregistreur(Node):
    def __init__(self, dossier: Path):
        super().__init__("enregistreur_video")
        self._dossier = dossier
        self._journal = (dossier / "evenements.jsonl").open("a", encoding="utf-8")
        self._dernier_objet = 0.0
        self._derniere_image: dict[str, float] = {}
        self.create_subscription(Log, "/rosout", self._cb_log, 100)
        self.create_subscription(PoseStamped, "/object_position", self._cb_objet, 10)
        for topic, sous_dossier in (("/perception/debug", "camera"), ("/scene_cam/image_raw", "scene")):
            (dossier / sous_dossier).mkdir(parents=True, exist_ok=True)
            self.create_subscription(Image, topic, lambda m, d=sous_dossier: self._cb_image(m, d),
                                     qos_profile_sensor_data)
        self.get_logger().info(f"Enregistrement dans {dossier}")

    def _ecrire(self, evenement: dict) -> None:
        self._journal.write(json.dumps(evenement, ensure_ascii=False) + "\n")
        self._journal.flush()

    def _cb_log(self, msg: Log) -> None:
        if msg.name in NOEUDS_SUIVIS:
            self._ecrire({"t": time.time(), "type": "log", "noeud": msg.name, "msg": msg.msg})

    def _cb_objet(self, msg: PoseStamped) -> None:
        maintenant = time.time()
        if maintenant - self._dernier_objet < PERIODE_OBJET_S:
            return
        self._dernier_objet = maintenant
        p = msg.pose.position
        self._ecrire({"t": maintenant, "type": "objet", "x": p.x, "y": p.y, "z": p.z})

    def _cb_image(self, msg: Image, sous_dossier: str) -> None:
        maintenant = time.time()
        if maintenant - self._derniere_image.get(sous_dossier, 0.0) < PERIODE_IMAGE_S:
            return
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        self._derniere_image[sous_dossier] = maintenant
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, : msg.width]
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(self._dossier / sous_dossier / f"{maintenant:.3f}.jpg"), image,
                    [cv2.IMWRITE_JPEG_QUALITY, QUALITE_JPEG])


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage : enregistreur.py <dossier_sortie>")
    rclpy.init()
    noeud = Enregistreur(Path(sys.argv[1]))
    try:
        rclpy.spin(noeud)
    except KeyboardInterrupt:
        pass
    finally:
        noeud._journal.close()


if __name__ == "__main__":
    main()
