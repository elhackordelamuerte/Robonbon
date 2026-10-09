#!/usr/bin/env python3
"""Panneau web léger pour piloter le SO-ARM101 via /joint_command.

Remplace les `ros2 topic pub ... /joint_command` manuels : sliders par joint,
poses enregistrées (poses.json), lecture de séquences animées (interpolation
linéaire publiée à 30 Hz) et lancement du grab d'objet (services du brain).
Aucune dépendance hors rclpy + bibliothèque standard.

    python3 panel.py            # http://localhost:8080
    PANEL_PORT=9000 python3 panel.py

Respecte la règle du sujet : passe uniquement par /joint_command.
"""
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

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

# Grab d'objet : services Trigger exposés par brain_node et driver_node.
SERVICES_GRAB = {
    "pick": "/pick_and_place",
    "rest": "/go_to_rest",
    "reset": "/reset_object",
}
NOEUD_DRIVER = "/so101_driver"
NOEUD_BRAIN = "so101_brain"
PARAM_FAKE_PERCEPTION = "fake_perception"
DELAI_SERVICE_S = 5.0
NB_MESSAGES_BRAIN = 8

# Passage simulation -> bras réel (cf. docs/BRAS_REEL.md). Le panneau ne fait que VÉRIFIER :
# changer de mode impose de relancer les conteneurs depuis le Mac (scripts/start.sh).
FICHIER_CALIBRATION = Path("/calib/arm.json")  # calib/ du dépôt, monté par start.sh
PONT_SERIE = ("host.docker.internal", 8765)  # scripts/serial_bridge.py sur le Mac
DELAI_PONT_S = 0.5
PARAMS_DRIVER = ["use_sim", "port", "max_step_rad", "max_step_gripper_pct"]


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
        self.create_subscription(PoseStamped, "/object_position", self._cb_objet, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_boite, 10)
        self.create_subscription(Log, "/rosout", self._cb_rosout, 50)
        self._clients_grab = {cle: self.create_client(Trigger, nom) for cle, nom in SERVICES_GRAB.items()}
        self._client_set_param = self.create_client(SetParameters, f"{NOEUD_DRIVER}/set_parameters")
        self._client_get_param = self.create_client(GetParameters, f"{NOEUD_DRIVER}/get_parameters")
        self.etat: dict = {}
        self.mode_reel = False  # mis à jour par /api/mode
        self.objet: list | None = None
        self.boite: list | None = None
        self.recu_objet = self.recu_boite = 0.0  # time.monotonic() de réception
        self.messages_brain: list = []
        self._arret_lecture = threading.Event()
        self._thread_lecture: threading.Thread | None = None

    def _cb_etat(self, msg: JointState):
        etat = dict(zip(msg.name, msg.position, strict=False))
        if "gripper" in etat:
            bas, haut = PINCE_RAD
            etat["gripper"] = (etat["gripper"] - bas) / (haut - bas) * 100.0
        self.etat = etat

    def _cb_objet(self, msg: PoseStamped):
        p = msg.pose.position
        self.objet, self.recu_objet = [p.x, p.y, p.z], time.monotonic()

    def _cb_boite(self, msg: PoseStamped):
        p = msg.pose.position
        self.boite, self.recu_boite = [p.x, p.y, p.z], time.monotonic()

    def _cb_rosout(self, msg: Log):
        """Garde les derniers messages du brain (étapes du pick & place)."""
        if msg.name != NOEUD_BRAIN:
            return
        self.messages_brain = [*self.messages_brain, msg.msg][-NB_MESSAGES_BRAIN:]

    def _appeler(self, client, requete):
        """Appel de service bloquant pour le thread HTTP (rclpy.spin tourne ailleurs)."""
        if not client.wait_for_service(timeout_sec=DELAI_SERVICE_S):
            raise TimeoutError(f"service {client.srv_name} indisponible")
        futur = client.call_async(requete)
        fin = time.monotonic() + DELAI_SERVICE_S
        while not futur.done():
            if time.monotonic() > fin:
                raise TimeoutError(f"pas de réponse de {client.srv_name}")
            time.sleep(0.02)
        return futur.result()

    def grab(self, action: str) -> dict:
        """Déclenche pick / rest / reset ; coupe d'abord une animation du panneau."""
        if action not in self._clients_grab:
            raise ValueError(f"action inconnue : {action}")
        self.arreter()
        reponse = self._appeler(self._clients_grab[action], Trigger.Request())
        return {"succes": reponse.success, "message": reponse.message}

    def definir_fake_perception(self, actif: bool) -> dict:
        valeur = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=actif)
        requete = SetParameters.Request(parameters=[Parameter(name=PARAM_FAKE_PERCEPTION, value=valeur)])
        resultat = self._appeler(self._client_set_param, requete).results[0]
        return {"succes": resultat.successful, "message": resultat.reason}

    def lire_fake_perception(self) -> bool | None:
        """None si le driver ne répond pas (paramètre absent ou nœud arrêté)."""
        try:
            reponse = self._appeler(self._client_get_param, GetParameters.Request(names=[PARAM_FAKE_PERCEPTION]))
        except TimeoutError:
            return None
        valeurs = reponse.values
        return bool(valeurs[0].bool_value) if valeurs and valeurs[0].type == ParameterType.PARAMETER_BOOL else None

    def lire_params_driver(self) -> dict | None:
        """Paramètres utiles du driver (mode, port, limites de vitesse) ; None s'il ne répond pas."""
        try:
            reponse = self._appeler(self._client_get_param, GetParameters.Request(names=PARAMS_DRIVER))
        except TimeoutError:
            return None
        lecteurs = {ParameterType.PARAMETER_BOOL: "bool_value", ParameterType.PARAMETER_STRING: "string_value",
                    ParameterType.PARAMETER_DOUBLE: "double_value", ParameterType.PARAMETER_INTEGER: "integer_value"}
        return {nom: getattr(v, lecteurs[v.type]) if v.type in lecteurs else None
                for nom, v in zip(PARAMS_DRIVER, reponse.values, strict=False)}

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


def pont_joignable() -> bool:
    """Le pont série écoute-t-il sur le Mac ? (connexion TCP refermée aussitôt)"""
    try:
        with socket.create_connection(PONT_SERIE, timeout=DELAI_PONT_S):
            return True
    except OSError:
        return False


def resume_calibration() -> dict | None:
    """Moteurs présents dans calib/arm.json, ou None si le fichier est absent ou illisible."""
    try:
        return {"moteurs": sorted(json.loads(FICHIER_CALIBRATION.read_text()))}
    except (OSError, ValueError):
        return None


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
            elif self.path == "/api/mode":
                driver = noeud.lire_params_driver()
                noeud.mode_reel = bool(driver) and driver.get("use_sim") is False
                # Bras réel : le pont n'accepte qu'un client, déjà occupé par le driver (socat) ;
                # une sonde TCP échouerait et s'empilerait dans sa file d'attente.
                pont = "driver" if noeud.mode_reel else pont_joignable()
                self._repondre(200, {"driver": driver, "calibration": resume_calibration(), "pont": pont})
            elif self.path == "/api/grab":
                maintenant = time.monotonic()
                self._repondre(200, {"objet": noeud.objet, "boite": noeud.boite,
                                     "age_objet": maintenant - noeud.recu_objet if noeud.objet else None,
                                     "age_boite": maintenant - noeud.recu_boite if noeud.boite else None,
                                     "messages": noeud.messages_brain,
                                     # paramètre propre à la sim : ne pas interroger un driver réel
                                     "fake_perception": None if noeud.mode_reel else noeud.lire_fake_perception()})
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
                elif self.path == "/api/grab":
                    self._repondre(200, noeud.grab(str(corps.get("action", ""))))
                    return
                elif self.path == "/api/perception":
                    self._repondre(200, noeud.definir_fake_perception(bool(corps.get("actif"))))
                    return
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
            except TimeoutError as erreur:
                self._repondre(503, {"erreur": str(erreur)})

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
