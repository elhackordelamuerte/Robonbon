#!/usr/bin/env python3
"""Panneau web léger pour piloter le SO-ARM101 via /joint_command.

Remplace les `ros2 topic pub ... /joint_command` manuels : sliders par joint,
poses enregistrées (poses.json) et lecture de séquences animées (interpolation
linéaire publiée à 30 Hz). Aucune dépendance hors rclpy + bibliothèque standard.

    python3 panel.py            # http://localhost:8080
    PANEL_PORT=9000 python3 panel.py

Respecte la règle du sujet : passe uniquement par /joint_command.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DOSSIER = Path(__file__).resolve().parent
FICHIER_HTML = DOSSIER / "index.html"
FICHIER_POSES = DOSSIER / "poses.json"
PORT = int(os.environ.get("PANEL_PORT", "8080"))
FREQUENCE_LECTURE_HZ = 30.0
DUREE_MIN_S = 0.1
TAILLE_MAX_REQUETE = 1_000_000

# Limites reprises de so101_description/urdf/so101.urdf (rad). Pince : 0-100 %.
LIMITES = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (0.0, 100.0),
}
PINCE_RAD = (-0.174533, 1.74533)  # même conversion que driver_node


def borner(nom: str, valeur: float) -> float:
    bas, haut = LIMITES[nom]
    return min(max(float(valeur), bas), haut)


def nettoyer_joints(joints: dict) -> dict:
    """Garde les joints connus, valeurs numériques bornées aux limites."""
    return {nom: borner(nom, val) for nom, val in joints.items() if nom in LIMITES}


class PanneauNode(Node):
    """Publie /joint_command et mémorise le dernier /joint_states (pince en %)."""

    def __init__(self):
        super().__init__("joint_panel")
        self._pub = self.create_publisher(JointState, "/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self._cb_etat, 10)
        self.etat: dict = {}
        self._arret_lecture = threading.Event()
        self._thread_lecture: threading.Thread | None = None

    def _cb_etat(self, msg: JointState):
        etat = dict(zip(msg.name, msg.position, strict=False))
        if "gripper" in etat:
            bas, haut = PINCE_RAD
            etat["gripper"] = (etat["gripper"] - bas) / (haut - bas) * 100.0
        self.etat = etat

    def publier(self, joints: dict):
        if not joints:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joints)
        msg.position = [float(v) for v in joints.values()]
        self._pub.publish(msg)

    @property
    def en_lecture(self) -> bool:
        return self._thread_lecture is not None and self._thread_lecture.is_alive()

    def lire_sequence(self, poses: list, boucle: bool):
        self.arreter()
        self._arret_lecture.clear()
        self._thread_lecture = threading.Thread(
            target=self._jouer, args=(poses, boucle), daemon=True
        )
        self._thread_lecture.start()

    def arreter(self):
        self._arret_lecture.set()
        if self.en_lecture:
            self._thread_lecture.join(timeout=1.0)

    def _jouer(self, poses: list, boucle: bool):
        """Interpole linéairement de la position courante vers chaque pose."""
        periode = 1.0 / FREQUENCE_LECTURE_HZ
        while True:
            for pose in poses:
                cible = pose["joints"]
                depart = {n: self.etat.get(n, cible[n]) for n in cible}
                duree = max(float(pose.get("duree", 1.0)), DUREE_MIN_S)
                t0 = time.monotonic()
                while True:
                    if self._arret_lecture.is_set():
                        return
                    a = min((time.monotonic() - t0) / duree, 1.0)
                    self.publier({n: depart[n] + a * (cible[n] - depart[n]) for n in cible})
                    if a >= 1.0:
                        break
                    time.sleep(periode)
            if not boucle:
                return


def charger_poses() -> dict:
    if not FICHIER_POSES.is_file():
        return {"poses": {}, "sequences": {}}
    return json.loads(FICHIER_POSES.read_text())


def fabriquer_handler(noeud: PanneauNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # silencieux
            pass

        def _repondre(self, code: int, corps, type_mime="application/json"):
            donnees = corps if isinstance(corps, bytes) else json.dumps(corps).encode()
            self.send_response(code)
            self.send_header("Content-Type", type_mime)
            self.send_header("Content-Length", str(len(donnees)))
            self.end_headers()
            self.wfile.write(donnees)

        def _lire_json(self):
            taille = int(self.headers.get("Content-Length", 0))
            if taille > TAILLE_MAX_REQUETE:
                raise ValueError("requête trop grande")
            return json.loads(self.rfile.read(taille) or b"{}")

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._repondre(200, FICHIER_HTML.read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/etat":
                self._repondre(200, {"etat": noeud.etat, "limites": LIMITES,
                                     "en_lecture": noeud.en_lecture})
            elif self.path == "/api/poses":
                self._repondre(200, charger_poses())
            else:
                self._repondre(404, {"erreur": "introuvable"})

        def do_POST(self):
            try:
                corps = self._lire_json()
                if self.path == "/api/commande":
                    noeud.arreter()
                    noeud.publier(nettoyer_joints(corps.get("joints", {})))
                elif self.path == "/api/lecture":
                    poses = [{"joints": nettoyer_joints(p["joints"]), "duree": p.get("duree", 1.0)}
                             for p in corps.get("poses", []) if p.get("joints")]
                    if not poses:
                        raise ValueError("séquence vide")
                    noeud.lire_sequence(poses, bool(corps.get("boucle", False)))
                elif self.path == "/api/stop":
                    noeud.arreter()
                elif self.path == "/api/poses":
                    if not isinstance(corps.get("poses"), dict) or not isinstance(corps.get("sequences"), dict):
                        raise ValueError("format attendu : {poses: {}, sequences: {}}")
                    FICHIER_POSES.write_text(json.dumps(corps, indent=2, ensure_ascii=False))
                else:
                    self._repondre(404, {"erreur": "introuvable"})
                    return
                self._repondre(200, {"ok": True})
            except (ValueError, KeyError, TypeError) as erreur:
                self._repondre(400, {"erreur": str(erreur)})

    return Handler


def main():
    rclpy.init()
    noeud = PanneauNode()
    threading.Thread(target=rclpy.spin, args=(noeud,), daemon=True).start()
    serveur = ThreadingHTTPServer(("0.0.0.0", PORT), fabriquer_handler(noeud))
    noeud.get_logger().info(f"Panneau prêt sur http://localhost:{PORT}")
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        noeud.arreter()
        serveur.server_close()
        noeud.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
