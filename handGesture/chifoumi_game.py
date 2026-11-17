# /// script
# requires-python = ">=3.11, <3.13"
# dependencies = [
#     "opencv-python==4.10.0.84",
#     "mediapipe==0.10.14",
#     "numpy<2",
#     "requests",
# ]
# ///
"""Jeu interactif Pierre - Feuille - Ciseaux (Chifoumi) contre le SO-ARM101.

Ce programme synchronise la détection en temps réel de votre main (via webcam)
avec les mouvements du bras robotisé en simulation (MuJoCo / RViz).

Règles :
  - 1... 2... 3... CHIFOUMI !
  - À 3, le joueur et le robot abattent leur coup en même temps.
  - L'arbitre compare les gestes et met à jour les scores.

Commandes clavier :
  - ESPACE : Lancer une nouvelle manche
  - 'A'    : Activer / Désactiver le mode automatique (enchaînement continu)
  - 'R'    : Réinitialiser les scores
  - 'C'    : Replacer le bras en garde (READY)
  - 'Q'    : Quitter
"""
from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

# Permet d'importer handGesture quel que soit le dossier de travail courant
_RACINE = Path(__file__).resolve().parent.parent
if str(_RACINE) not in sys.path:
    sys.path.insert(0, str(_RACINE))

import cv2
import numpy as np

from handGesture.hand_detector import COULEURS_GESTES, GESTES, HandDetector
from handGesture.robot_chifoumi import RobotChifoumi

# États du jeu
ETAT_ATTENTE = "ATTENTE"
ETAT_COMPTE = "COMPTE"
ETAT_VERDICT = "VERDICT"

# Choix possibles
CHOIX_POSSIBLES = ["PIERRE", "FEUILLE", "CISEAUX"]

# Règles de victoire : CLE bat VALEUR
REGLES_VICTOIRE = {
    "PIERRE": "CISEAUX",
    "CISEAUX": "FEUILLE",
    "FEUILLE": "PIERRE",
}

EMOJIS = {
    "PIERRE": "🪨",
    "FEUILLE": "📄",
    "CISEAUX": "✂️",
    "AUCUNE": "❓",
    "INCONNU": "❓",
}


def determiner_vainqueur(coup_humain: str, coup_robot: str) -> str:
    """Retourne 'HUMAIN', 'ROBOT' ou 'EGALITE'."""
    if coup_humain == coup_robot:
        return "EGALITE"
    if REGLES_VICTOIRE.get(coup_humain) == coup_robot:
        return "HUMAIN"
    return "ROBOT"


class ChifoumiGame:
    """Orchestrateur du jeu Chifoumi entre l'humain et le robot."""

    def __init__(
        self,
        camera_index: int = 0,
        camera_source: str = "",
        panel_url: str = "http://localhost:8080",
        hold_duration: float = 6.0,
    ):
        self.detector = HandDetector()
        self.robot = RobotChifoumi(panel_url=panel_url)

        self.backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
        self.src = camera_source if camera_source else camera_index

        self.etat = ETAT_ATTENTE
        self.temps_debut_etat = time.time()
        self.mode_auto = False

        # Durée de maintien de la pose du robot (5 à 10 secondes)
        self.duree_maintien_pose = max(1.0, float(hold_duration))

        # Scores
        self.score_humain = 0
        self.score_robot = 0
        self.score_egalites = 0
        self.manches_jouees = 0

        # Résultats de la dernière manche
        self.dernier_coup_humain = ""
        self.dernier_coup_robot = ""
        self.dernier_vainqueur = ""

        # Décompte
        self.etape_compte = 0  # 1, 2 ou 3
        self.duree_etape = 0.8  # secondes par étape de balancement

    def lancer_manche(self):
        """Démarre le compte à rebours synchronisé."""
        if self.etat != ETAT_ATTENTE:
            return
        self.etat = ETAT_COMPTE
        self.etape_compte = 1
        self.temps_debut_etat = time.time()
        # Le robot balance son bras pour le 1
        self.robot.animer_compte_a_rebours(1)

    def evaluer_manche(self, geste_humain_capture: str):
        """Valide le coup du joueur et du robot, et arbitre."""
        # Coup aléatoire du robot
        coup_robot = random.choice(CHOIX_POSSIBLES)
        self.dernier_coup_robot = coup_robot
        # Le robot joue son coup et le MAINTIENT fermement dans la simulation
        self.robot.jouer_coup(coup_robot, duree=0.5)

        # Coup de l'humain (si non reconnu, on prend le dernier geste valide ou faute de jeu)
        if geste_humain_capture in CHOIX_POSSIBLES:
            coup_humain = geste_humain_capture
        else:
            coup_humain = "INCONNU"

        self.dernier_coup_humain = coup_humain
        self.manches_jouees += 1

        if coup_humain not in CHOIX_POSSIBLES:
            self.dernier_vainqueur = "ROBOT"
            self.score_robot += 1
        else:
            vainqueur = determiner_vainqueur(coup_humain, coup_robot)
            self.dernier_vainqueur = vainqueur
            if vainqueur == "HUMAIN":
                self.score_humain += 1
            elif vainqueur == "ROBOT":
                self.score_robot += 1
            else:
                self.score_egalites += 1

        self.etat = ETAT_VERDICT
        self.temps_debut_etat = time.time()

    def update(self, geste_actuel: str):
        """Machine d'états temporelle du jeu."""
        maintenant = time.time()
        temps_dans_etat = maintenant - self.temps_debut_etat

        if self.etat == ETAT_COMPTE:
            if self.etape_compte == 1 and temps_dans_etat >= self.duree_etape:
                self.etape_compte = 2
                self.temps_debut_etat = maintenant
                self.robot.animer_compte_a_rebours(2)
            elif self.etape_compte == 2 and temps_dans_etat >= self.duree_etape:
                self.etape_compte = 3
                self.temps_debut_etat = maintenant
                self.robot.animer_compte_a_rebours(3)
            elif self.etape_compte == 3 and temps_dans_etat >= self.duree_etape:
                # INSTANT CRUCIAL : "CHIFOUMI !"
                # On capture le geste de l'humain au moment exact du tir
                self.evaluer_manche(geste_actuel)

        elif self.etat == ETAT_VERDICT and temps_dans_etat >= self.duree_maintien_pose:
            # Fin du temps de maintien (5 à 10 secondes) : retour fluide en garde
            self.etat = ETAT_ATTENTE
            self.robot.aller_en_garde(duree=1.0)
            self.temps_debut_etat = maintenant
            if self.mode_auto:
                # En mode auto, pause de 1.5 s en garde avant de relancer
                time.sleep(1.5)
                self.lancer_manche()

    def dessiner_hud(self, frame: np.ndarray, geste_actuel: str) -> np.ndarray:
        """Incruste les informations de jeu modernes sur l'image."""
        h, w = frame.shape[:2]
        hud = frame.copy()

        # 1. BANDEAU SUPÉRIEUR (Scores & État)
        overlay = hud.copy()
        cv2.rectangle(overlay, (0, 0), (w, 75), (20, 20, 25), -1)
        cv2.addWeighted(overlay, 0.85, hud, 0.15, 0, hud)

        # Titre et scores
        cv2.putText(hud, "CHIFOUMI SO-ARM101", (25, 30), cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 215, 255), 2, cv2.LINE_AA)
        statut_robot = "Robot: Connecté ✅" if self.robot._en_ligne else "Robot: Non connecté (Start Docker)"
        cv2.putText(hud, statut_robot, (25, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

        # Tableau des scores au centre droit
        score_text = f"JOUEUR: {self.score_humain}   |   ROBOT: {self.score_robot}   |   NULS: {self.score_egalites}"
        cv2.putText(hud, score_text, (w // 2 - 150, 45), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # Mode auto
        mode_txt = "Mode: AUTO [A]" if self.mode_auto else "Mode: MANUEL [A]"
        cv2.putText(hud, mode_txt, (w - 180, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        # 2. BANDEAU INFÉRIEUR (Geste en cours & Raccourcis)
        overlay_bottom = hud.copy()
        cv2.rectangle(overlay_bottom, (0, h - 55), (w, h), (20, 20, 25), -1)
        cv2.addWeighted(overlay_bottom, 0.85, hud, 0.15, 0, hud)

        # Geste humain en direct
        nom_geste = GESTES.get(geste_actuel, geste_actuel)
        c_geste = COULEURS_GESTES.get(geste_actuel, (255, 255, 255))
        cv2.putText(hud, f"Main: {nom_geste}", (25, h - 20), cv2.FONT_HERSHEY_DUPLEX, 0.7, c_geste, 2, cv2.LINE_AA)

        # Raccourcis
        aide = "ESPACE: Jouer  |  R: Reset  |  C: En Garde  |  Q: Quitter"
        cv2.putText(hud, aide, (w // 2 - 120, h - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)

        # 3. OVERLAY CENTRAL SELON L'ÉTAT
        if self.etat == ETAT_COMPTE:
            # Animation du compte à rebours
            txt_compte = ""
            if self.etape_compte == 1:
                txt_compte = "1... PIERRE !"
            elif self.etape_compte == 2:
                txt_compte = "2... FEUILLE !"
            elif self.etape_compte == 3:
                txt_compte = "3... CISEAUX !"

            # Cartouche central
            (tw, th), _ = cv2.getTextSize(txt_compte, cv2.FONT_HERSHEY_DUPLEX, 1.8, 3)
            cx, cy = w // 2, h // 2
            cv2.rectangle(hud, (cx - tw // 2 - 30, cy - th - 30), (cx + tw // 2 + 30, cy + 30), (0, 0, 0), -1)
            cv2.rectangle(hud, (cx - tw // 2 - 30, cy - th - 30), (cx + tw // 2 + 30, cy + 30), (0, 215, 255), 3)
            cv2.putText(hud, txt_compte, (cx - tw // 2, cy), cv2.FONT_HERSHEY_DUPLEX, 1.8, (255, 255, 255), 3, cv2.LINE_AA)

        elif self.etat == ETAT_VERDICT:
            # Affichage du résultat de la manche
            cx, cy = w // 2, h // 2

            # Cartouche de résultat
            if self.dernier_vainqueur == "HUMAIN":
                titre = "VICTOIRE ! 🎉"
                c_titre = (50, 205, 50)  # Vert
            elif self.dernier_vainqueur == "ROBOT":
                titre = "LE ROBOT GAGNE ! 🤖"
                c_titre = (0, 0, 255)  # Rouge
            else:
                titre = "ÉGALITÉ ! 🤝"
                c_titre = (0, 215, 255)  # Jaune/Or

            detail = f"Vous : {self.dernier_coup_humain}  vs  Robot : {self.dernier_coup_robot}"

            (tw1, _th1), _ = cv2.getTextSize(titre, cv2.FONT_HERSHEY_DUPLEX, 1.6, 3)
            (tw2, _th2), _ = cv2.getTextSize(detail, cv2.FONT_HERSHEY_DUPLEX, 0.9, 2)

            temps_restant = max(0.0, self.duree_maintien_pose - (time.time() - self.temps_debut_etat))
            txt_maintien = f"Pose maintenue dans RViz (retour garde: {temps_restant:.1f}s)"
            (tw3, _th3), _ = cv2.getTextSize(txt_maintien, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)

            largeur_box = max(tw1, tw2, tw3) + 60
            hauteur_box = 175

            overlay_box = hud.copy()
            cv2.rectangle(
                overlay_box,
                (cx - largeur_box // 2, cy - hauteur_box // 2),
                (cx + largeur_box // 2, cy + hauteur_box // 2),
                (20, 20, 20),
                -1,
            )
            cv2.addWeighted(overlay_box, 0.9, hud, 0.1, 0, hud)
            cv2.rectangle(
                hud,
                (cx - largeur_box // 2, cy - hauteur_box // 2),
                (cx + largeur_box // 2, cy + hauteur_box // 2),
                c_titre,
                3,
            )

            cv2.putText(hud, titre, (cx - tw1 // 2, cy - 25), cv2.FONT_HERSHEY_DUPLEX, 1.6, c_titre, 3, cv2.LINE_AA)
            cv2.putText(hud, detail, (cx - tw2 // 2, cy + 20), cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(hud, txt_maintien, (cx - tw3 // 2, cy + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 220, 255), 1, cv2.LINE_AA)

        elif self.etat == ETAT_ATTENTE and self.manches_jouees == 0:
            # Message d'accueil au premier lancement
            msg = "Prêt ? Appuyez sur [ESPACE] pour jouer !"
            (tw, th), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_DUPLEX, 0.9, 2)
            cx, cy = w // 2, h // 2 + 100
            cv2.rectangle(hud, (cx - tw // 2 - 20, cy - th - 15), (cx + tw // 2 + 20, cy + 15), (30, 30, 30), -1)
            cv2.rectangle(hud, (cx - tw // 2 - 20, cy - th - 15), (cx + tw // 2 + 20, cy + 15), (0, 215, 255), 2)
            cv2.putText(hud, msg, (cx - tw // 2, cy), cv2.FONT_HERSHEY_DUPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

        return hud

    def run(self):
        """Boucle principale du jeu."""
        print(f"📷 Ouverture de la caméra ({self.src})...")
        cap = cv2.VideoCapture(self.src, self.backend) if isinstance(self.src, int) else cv2.VideoCapture(self.src)

        if not cap.isOpened():
            print(f"❌ Impossible d'ouvrir la caméra {self.src}")
            return

        # Vérifier connexion robot
        if self.robot.tester_connexion():
            print("🤖 Robot connecté : mise en garde...")
            self.robot.aller_en_garde()
        else:
            print("⚠️ Robot non connecté (Docker non lancé ou port 8080 injoignable).")
            print("   Le jeu fonctionne en mode visuel autonome ; lancez `./scripts/start.sh` pour voir le bras bouger !")

        print("\n🎮 Jeu prêt ! Appuyez sur ESPACE dans la fenêtre vidéo pour jouer.\n")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 30)

        fps_timer = time.time()
        fps_compteur = 0
        fps_affiche = 0.0
        echecs_consecutifs = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                echecs_consecutifs += 1
                if echecs_consecutifs > 20:
                    print("⚠️ Erreur de lecture vidéo (trop d'échecs consécutifs).")
                    break
                time.sleep(0.05)
                continue
            echecs_consecutifs = 0

            # Miroir horizontal
            frame = cv2.flip(frame, 1)

            # Traitement de la main
            geste, doigts, landmarks = self.detector.traiter_image(frame)
            annotated = self.detector.dessiner_annotations(frame, landmarks, geste, doigts)

            # Mise à jour du jeu
            self.update(geste)

            # Rendu de l'interface
            affichage = self.dessiner_hud(annotated, geste)

            # FPS
            fps_compteur += 1
            if time.time() - fps_timer >= 1.0:
                fps_affiche = fps_compteur / (time.time() - fps_timer)
                fps_compteur = 0
                fps_timer = time.time()

            cv2.putText(
                affichage,
                f"{fps_affiche:.0f} FPS",
                (affichage.shape[1] - 80, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (100, 255, 100),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow("Chifoumi SO-ARM101 - Pierre Feuille Ciseaux", affichage)

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord("q"), 27):  # 'q' ou Échap
                break
            elif touche == ord(" "):
                self.lancer_manche()
            elif touche in (ord("a"), ord("A")):
                self.mode_auto = not self.mode_auto
                if self.mode_auto and self.etat == ETAT_ATTENTE:
                    self.lancer_manche()
            elif touche in (ord("r"), ord("R")):
                self.score_humain = 0
                self.score_robot = 0
                self.score_egalites = 0
                self.manches_jouees = 0
                print("🔄 Scores réinitialisés.")
            elif touche in (ord("c"), ord("C")):
                self.robot.aller_en_garde()
                print("🦾 Robot remis en garde.")

        cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Jeu Pierre-Feuille-Ciseaux contre le bras SO-ARM101")
    parser.add_argument("--index", type=int, default=0, help="Index webcam (défaut: 0)")
    parser.add_argument("--source", type=str, default="", help="Source vidéo alternative (URL flux)")
    parser.add_argument("--panel-url", type=str, default="http://localhost:8080", help="URL du panneau ROS 2")
    parser.add_argument("--hold", "--duree-pose", type=float, default=6.0, help="Durée de maintien de la pose (s, défaut: 6.0)")
    args = parser.parse_args()

    jeu = ChifoumiGame(
        camera_index=args.index,
        camera_source=args.source,
        panel_url=args.panel_url,
        hold_duration=args.hold,
    )
    jeu.run()


if __name__ == "__main__":
    main()
