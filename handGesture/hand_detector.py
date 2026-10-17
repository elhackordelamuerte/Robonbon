# /// script
# requires-python = ">=3.11, <3.13"
# dependencies = [
#     "opencv-python==4.10.0.84",
#     "mediapipe==0.10.14",
#     "numpy<2",
# ]
# ///
"""Détecteur de main et classifieur de gestes Pierre - Feuille - Ciseaux.

Utilise MediaPipe Hands pour détecter les 21 points clés de la main et
classifier le geste en temps réel de manière robuste :
  - PIERRE  : tous les doigts repliés (poing fermé)
  - FEUILLE : tous les doigts dépliés (main ouverte)
  - CISEAUX : index et majeur dépliés, annulaire et auriculaire repliés
"""
from __future__ import annotations

import math
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np

GESTES = {
    "PIERRE": "Pierre 🪨",
    "FEUILLE": "Feuille 📄",
    "CISEAUX": "Ciseaux ✂️",
    "INCONNU": "En attente...",
    "AUCUNE": "Aucune main",
}

COULEURS_GESTES = {
    "PIERRE": (255, 120, 30),    # Bleu / Cyan BGR
    "FEUILLE": (50, 205, 50),     # Vert
    "CISEAUX": (0, 165, 255),     # Orange
    "INCONNU": (180, 180, 180),   # Gris
    "AUCUNE": (100, 100, 100),    # Gris foncé
}


class HandDetector:
    """Détecte les mains et classifie le geste Pierre / Feuille / Ciseaux."""

    def __init__(
        self,
        max_num_hands: int = 1,
        min_detection_confidence: float = 0.7,
        min_tracking_confidence: float = 0.6,
        taille_historique: int = 5,
    ):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_draw_styles = mp.solutions.drawing_styles

        # Historique pour stabilisation temporelle (vote majoritaire)
        self._historique: deque[str] = deque(maxlen=taille_historique)

    def detecter_doigts_leves(self, landmarks, handedness: str = "Right") -> list[int]:
        """Retourne une liste de 5 entiers (0 ou 1) indiquant si chaque doigt est déplié.
        Ordre: [Pouce, Index, Majeur, Annulaire, Auriculaire]
        """
        doigts = [0, 0, 0, 0, 0]
        pts = landmarks.landmark

        # Poignet
        poignet = pts[0]

        # 1. POUCE
        # Pour le pouce, on vérifie si le bout (point 4) s'éloigne de la base de l'auriculaire (point 17)
        # et de l'articulation MCP du pouce (point 2).
        d_pouce_tip_auriculaire = math.hypot(pts[4].x - pts[17].x, pts[4].y - pts[17].y)
        d_pouce_ip_auriculaire = math.hypot(pts[3].x - pts[17].x, pts[3].y - pts[17].y)
        d_pouce_mcp = math.hypot(pts[4].x - pts[2].x, pts[4].y - pts[2].y)

        # Si le bout du pouce est plus éloigné que son articulation intermédiaire
        if d_pouce_tip_auriculaire > d_pouce_ip_auriculaire * 1.15 and d_pouce_mcp > 0.08:
            doigts[0] = 1

        # 2. LES 4 AUTRES DOIGTS (Index: 8/6, Majeur: 12/10, Annulaire: 16/14, Auriculaire: 20/18)
        # Un doigt est levé si la distance entre son extrémité (TIP) et le poignet (WRIST)
        # est significativement supérieure à la distance entre son articulation PIP et le poignet.
        bout_indices = [8, 12, 16, 20]
        pip_indices = [6, 10, 14, 18]
        mcp_indices = [5, 9, 13, 17]

        for i, (bout, pip, mcp) in enumerate(zip(bout_indices, pip_indices, mcp_indices)):
            dist_bout_poignet = math.hypot(pts[bout].x - poignet.x, pts[bout].y - poignet.y)
            dist_pip_poignet = math.hypot(pts[pip].x - poignet.x, pts[pip].y - poignet.y)
            dist_bout_mcp = math.hypot(pts[bout].x - pts[mcp].x, pts[bout].y - pts[mcp].y)
            dist_pip_mcp = math.hypot(pts[pip].x - pts[mcp].x, pts[pip].y - pts[mcp].y)

            # Le doigt est considéré étendu si son bout est plus loin du poignet que son PIP
            # et si le segment MCP->TIP est nettement plus grand que MCP->PIP
            if dist_bout_poignet > dist_pip_poignet * 1.15 and dist_bout_mcp > dist_pip_mcp * 1.3:
                doigts[i + 1] = 1

        return doigts

    def classifier_geste(self, doigts: list[int]) -> str:
        """Classifie le geste à partir de l'état des 5 doigts [P, I, M, A, Au]."""
        _pouce, index, majeur, annulaire, auriculaire = doigts
        nb_doigts_leves = sum(doigts)
        nb_doigts_hors_pouce = sum(doigts[1:])

        # PIERRE : aucun ou presque aucun doigt levé (poing fermé)
        if nb_doigts_hors_pouce == 0:
            return "PIERRE"

        # CISEAUX : Index et Majeur levés, Annulaire et Auriculaire fermés
        # Le pouce peut être replié ou légèrement écarté selon la façon dont la personne fait les ciseaux
        if index == 1 and majeur == 1 and annulaire == 0 and auriculaire == 0:
            return "CISEAUX"

        # FEUILLE : 4 ou 5 doigts levés (main ouverte)
        if nb_doigts_hors_pouce >= 3 and (nb_doigts_leves >= 4 or nb_doigts_hors_pouce == 4):
            return "FEUILLE"

        # Cas particuliers fréquents :
        # - Ciseaux avec pouce ouvert (forme de L ou pistolet à 3 doigts parfois utilisé)
        if index == 1 and majeur == 1 and annulaire == 0 and auriculaire == 0:
            return "CISEAUX"

        # Si index seul ou auriculaire seul -> non reconnu comme coup légal
        return "INCONNU"

    def traiter_image(self, bgr_image: np.ndarray) -> tuple[str, list[int], any | None]:
        """Traite une image BGR et retourne :
        - Le geste stabilisé (PIERRE, FEUILLE, CISEAUX, INCONNU, AUCUNE)
        - La liste des doigts levés [P, I, M, A, Au]
        - Les landmarks bruts de la main (ou None)
        """
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        resultats = self.hands.process(rgb_image)

        if not resultats.multi_hand_landmarks:
            self._historique.append("AUCUNE")
            return "AUCUNE", [0, 0, 0, 0, 0], None

        landmarks = resultats.multi_hand_landmarks[0]
        handedness = "Right"
        if resultats.multi_handedness:
            handedness = resultats.multi_handedness[0].classification[0].label

        doigts = self.detecter_doigts_leves(landmarks, handedness)
        geste_brut = self.classifier_geste(doigts)
        self._historique.append(geste_brut)

        # Vote majoritaire sur l'historique récent
        geste_stabilise = max(set(self._historique), key=self._historique.count)

        return geste_stabilise, doigts, landmarks

    def dessiner_annotations(
        self,
        image: np.ndarray,
        landmarks,
        geste: str,
        doigts: list[int],
    ) -> np.ndarray:
        """Dessine les points du squelette de la main et les labels sur l'image."""
        h, w, _ = image.shape
        annotated = image.copy()

        if landmarks is not None:
            # Squelette MediaPipe
            self.mp_draw.draw_landmarks(
                annotated,
                landmarks,
                self.mp_hands.HAND_CONNECTIONS,
                self.mp_draw_styles.get_default_hand_landmarks_style(),
                self.mp_draw_styles.get_default_hand_connections_style(),
            )

            # Boîte englobante
            x_coords = [int(pt.x * w) for pt in landmarks.landmark]
            y_coords = [int(pt.y * h) for pt in landmarks.landmark]
            x_min, x_max = max(0, min(x_coords) - 20), min(w, max(x_coords) + 20)
            y_min, y_max = max(0, min(y_coords) - 20), min(h, max(y_coords) + 20)

            couleur = COULEURS_GESTES.get(geste, (200, 200, 200))
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), couleur, 2)

            # Badge geste
            texte_badge = GESTES.get(geste, geste)
            (tw, _th), _ = cv2.getTextSize(texte_badge, cv2.FONT_HERSHEY_DUPLEX, 0.7, 2)
            cv2.rectangle(
                annotated,
                (x_min, max(0, y_min - 30)),
                (x_min + tw + 16, max(0, y_min)),
                couleur,
                -1,
            )
            cv2.putText(
                annotated,
                texte_badge,
                (x_min + 8, max(20, y_min - 8)),
                cv2.FONT_HERSHEY_DUPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return annotated


def main():
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="Test du détecteur de main (Pierre-Feuille-Ciseaux)")
    parser.add_argument("--index", type=int, default=0, help="Index de la webcam (défaut: 0)")
    parser.add_argument("--source", type=str, default="", help="URL ou chemin vidéo (optionnel)")
    args = parser.parse_args()

    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    src = args.source if args.source else args.index

    print(f"📷 Ouverture de la caméra : {src}...")
    cap = cv2.VideoCapture(src, backend) if isinstance(src, int) else cv2.VideoCapture(src)

    if not cap.isOpened():
        print(f"❌ Impossible d'ouvrir la caméra {src}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    detector = HandDetector()
    print("✅ Détecteur prêt ! Appuyez sur 'q' pour quitter.")

    echecs_consecutifs = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            echecs_consecutifs += 1
            if echecs_consecutifs > 20:
                print("⚠️ Fin du flux ou caméra déconnectée.")
                break
            time.sleep(0.05)
            continue
        echecs_consecutifs = 0

        # Miroir horizontal pour une interaction naturelle (mode selfie)
        frame = cv2.flip(frame, 1)

        geste, doigts, landmarks = detector.traiter_image(frame)
        annotated = detector.dessiner_annotations(frame, landmarks, geste, doigts)

        # Affichage du statut en haut
        cv2.rectangle(annotated, (10, 10), (320, 60), (30, 30, 30), -1)
        cv2.putText(
            annotated,
            f"Geste: {GESTES.get(geste, geste)}",
            (20, 42),
            cv2.FONT_HERSHEY_DUPLEX,
            0.7,
            COULEURS_GESTES.get(geste, (255, 255, 255)),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow("Test Detection Pierre-Feuille-Ciseaux (q pour quitter)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
