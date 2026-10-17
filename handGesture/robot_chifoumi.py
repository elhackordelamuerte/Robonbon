# /// script
# requires-python = ">=3.11, <3.13"
# dependencies = [
#     "requests",
# ]
# ///
"""Contrôleur du bras SO-ARM101 pour le jeu Pierre - Feuille - Ciseaux.

Communique avec la stack ROS 2 (via l'API HTTP du panneau joint_panel
ou directement via ROS 2) pour exécuter :
  - La mise en garde (READY)
  - Le compte à rebours dynamique (3 balancements rythmés "1... 2... 3...")
  - Les poses de jeu (PIERRE, FEUILLE, CISEAUX)
  - Les animations de fin de manche (célébration de victoire, dépit lors d'une défaite)
"""
from __future__ import annotations

import time

import requests

PANEL_URL_DEFAUT = "http://localhost:8080"

# Définition des poses articulaires (radians, pince en %)
# Joints : shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
POSES_CHIFOUMI = {
    # Position neutre / d'attente
    "READY": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.50,
        "elbow_flex": 0.70,
        "wrist_flex": 0.40,
        "wrist_roll": 0.0,
        "gripper": 15.0,
    },
    # Balancement vers le haut (rythme 1, 2, 3)
    "PUMP_UP": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.75,
        "elbow_flex": 0.55,
        "wrist_flex": 0.65,
        "wrist_roll": 0.0,
        "gripper": 10.0,
    },
    # Balancement vers le bas
    "PUMP_DOWN": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.35,
        "elbow_flex": 0.85,
        "wrist_flex": 0.20,
        "wrist_roll": 0.0,
        "gripper": 10.0,
    },
    # PIERRE : Pince fermée hermétiquement, poing vers l'avant
    "PIERRE": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.42,
        "elbow_flex": 0.75,
        "wrist_flex": 0.35,
        "wrist_roll": 0.0,
        "gripper": 0.0,  # 0% = poing fermé
    },
    # FEUILLE : Pince grande ouverte à 100%, poignet à plat horizontal
    "FEUILLE": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.45,
        "elbow_flex": 0.65,
        "wrist_flex": 0.10,
        "wrist_roll": 1.57,  # horizontal
        "gripper": 100.0,   # 100% = main ouverte
    },
    # CISEAUX : Pince entrouverte à 45%, orientée comme des ciseaux
    "CISEAUX": {
        "shoulder_pan": 0.0,
        "shoulder_lift": -0.40,
        "elbow_flex": 0.70,
        "wrist_flex": 0.25,
        "wrist_roll": 1.57,
        "gripper": 45.0,    # ~45% = deux lames de ciseaux
    },
    # Repos
    "REPOS": {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.0,
    },
}


class RobotChifoumi:
    """Pilote les mouvements du bras pour le jeu Chifoumi."""

    def __init__(self, panel_url: str = PANEL_URL_DEFAUT):
        self.panel_url = panel_url.rstrip("/")
        self._en_ligne: bool | None = None

    def tester_connexion(self) -> bool:
        """Vérifie si le panneau web ROS 2 est accessible."""
        try:
            r = requests.get(f"{self.panel_url}/api/etat", timeout=1.5)
            self._en_ligne = r.status_code == 200
            return self._en_ligne
        except requests.RequestException:
            self._en_ligne = False
            return False

    def envoyer_pose(self, pose: dict[str, float]) -> bool:
        """Envoie immédiatement une consigne articulaire sur /joint_command via le panneau."""
        try:
            r = requests.post(
                f"{self.panel_url}/api/commande",
                json={"joints": pose},
                timeout=1.0,
            )
            return r.status_code == 200
        except requests.RequestException:
            return False

    def jouer_sequence(self, poses: list[dict], boucle: bool = False) -> bool:
        """Envoie une séquence avec interpolation fluide au panneau."""
        try:
            r = requests.post(
                f"{self.panel_url}/api/lecture",
                json={"poses": poses, "boucle": boucle},
                timeout=1.5,
            )
            return r.status_code == 200
        except requests.RequestException:
            return False

    def aller_en_garde(self, duree: float = 1.0) -> bool:
        """Place le bras en position d'attente (READY)."""
        return self.jouer_sequence([{"joints": POSES_CHIFOUMI["READY"], "duree": duree}])

    def animer_compte_a_rebours(self, pas: int) -> bool:
        """Anime un balancement du bras correspondant au numéro du compte à rebours (1, 2 ou 3).
        Haut puis bas pour rythmer le décompte.
        """
        seq = [
            {"joints": POSES_CHIFOUMI["PUMP_UP"], "duree": 0.35},
            {"joints": POSES_CHIFOUMI["PUMP_DOWN"], "duree": 0.35},
        ]
        return self.jouer_sequence(seq)

    def jouer_coup(self, coup: str, duree: float = 0.4) -> bool:
        """Exécute le coup choisi (PIERRE, FEUILLE ou CISEAUX)."""
        coup_maj = coup.upper()
        if coup_maj not in ("PIERRE", "FEUILLE", "CISEAUX"):
            raise ValueError(f"Coup inconnu : {coup}")

        pose = POSES_CHIFOUMI[coup_maj]
        return self.jouer_sequence([{"joints": pose, "duree": duree}])

    def celebrer_victoire(self) -> bool:
        """Petite danse de célébration : balance la pince avec des claquements."""
        seq = [
            {
                "joints": {
                    "shoulder_pan": 0.35,
                    "shoulder_lift": -0.6,
                    "elbow_flex": 0.6,
                    "wrist_flex": 0.5,
                    "wrist_roll": 0.0,
                    "gripper": 90.0,
                },
                "duree": 0.4,
            },
            {
                "joints": {
                    "shoulder_pan": -0.35,
                    "shoulder_lift": -0.6,
                    "elbow_flex": 0.6,
                    "wrist_flex": 0.5,
                    "wrist_roll": 0.0,
                    "gripper": 10.0,
                },
                "duree": 0.4,
            },
            {
                "joints": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": -0.5,
                    "elbow_flex": 0.7,
                    "wrist_flex": 0.4,
                    "wrist_roll": 0.0,
                    "gripper": 15.0,
                },
                "duree": 0.5,
            },
        ]
        return self.jouer_sequence(seq)

    def depit_defaite(self) -> bool:
        """Mouvement de dépit : tête baissée."""
        seq = [
            {
                "joints": {
                    "shoulder_pan": 0.0,
                    "shoulder_lift": -0.2,
                    "elbow_flex": 0.95,
                    "wrist_flex": -0.4,
                    "wrist_roll": 0.0,
                    "gripper": 5.0,
                },
                "duree": 0.6,
            },
            {
                "joints": POSES_CHIFOUMI["READY"],
                "duree": 0.8,
            },
        ]
        return self.jouer_sequence(seq)


def main():
    robot = RobotChifoumi()
    print("🤖 Test du contrôleur robot Chifoumi...")
    if robot.tester_connexion():
        print("✅ Connecté au panneau ROS 2 !")
        print("Mise en garde...")
        robot.aller_en_garde()
        time.sleep(1.2)
        print("Coup : PIERRE")
        robot.jouer_coup("PIERRE")
        time.sleep(1.5)
        print("Coup : FEUILLE")
        robot.jouer_coup("FEUILLE")
        time.sleep(1.5)
        print("Coup : CISEAUX")
        robot.jouer_coup("CISEAUX")
        time.sleep(1.5)
        print("Retour en garde...")
        robot.aller_en_garde()
    else:
        print("⚠️ Le panneau ROS 2 n'est pas lancé sur http://localhost:8080.")
        print("Lancez d'abord `./scripts/start.sh` pour piloter le bras en simulation.")


if __name__ == "__main__":
    main()
