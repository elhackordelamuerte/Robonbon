# Journal des modifications (Changelog)

Toutes les modifications notables apportées à ce projet sont consignées dans ce fichier.
Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/).

---

## [Unreleased] - 2026-10-17

### Ajouté
- **Module `handGesture/`** :
  - [`handGesture/hand_detector.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/hand_detector.py) : Détection de main en temps réel via MediaPipe Hands (21 landmarks) et classification géométrique des gestes Pierre, Feuille, Ciseaux avec lissage temporel (vote majoritaire). Compatible macOS (`AVFoundation`) et multi-plateforme.
  - [`handGesture/robot_chifoumi.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/robot_chifoumi.py) : Contrôleur du bras SO-ARM101 dédié au Chifoumi :
    - Définition des poses articulaires : `READY` (en garde), `PUMP_UP` / `PUMP_DOWN` (balancements rythmés 1-2-3), `PIERRE` (pince fermée à 0%), `FEUILLE` (pince ouverte à 100%, poignet à plat), `CISEAUX` (pince entrouverte à 45%, effet lames).
    - Animations de célébration (danse de victoire) et de dépit (tête baissée en cas de défaite).
    - Communication fluide avec ROS 2 via l'API REST de `joint_panel` (`/api/commande`, `/api/lecture`).
  - [`handGesture/chifoumi_game.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/chifoumi_game.py) : Application interactive complète avec affichage HUD moderne :
    - Retour vidéo webcam annoté avec squelette des mains.
    - Synchronisation milliseconde du coup du joueur et du coup du robot (1... 2... 3... Chifoumi !).
    - Arbitrage automatique, gestion des scores (Joueur vs Robot vs Égalités), historique des manches.
    - Modes Manuel (`ESPACE`) et Automatique (`A`).
  - [`handGesture/__init__.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/__init__.py) : Initialisation du package Python.

- **Scripts & Outils** :
  - [`scripts/chifoumi.sh`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/scripts/chifoumi.sh) : Script de lancement rapide en une commande (`./scripts/chifoumi.sh` ou `./scripts/chifoumi.sh --test`).
  - [`ros2_ws/tools/joint_panel/poses.json`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/ros2_ws/tools/joint_panel/poses.json) : Ajout des poses Chifoumi (`chifoumi_garde`, `chifoumi_pierre`, `chifoumi_feuille`, `chifoumi_ciseaux`) et d'une séquence de démonstration `chifoumi_demo` utilisable directement depuis le panneau web (`http://localhost:8080`).
  - [`ros2_ws/tools/joint_panel/index.html`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/ros2_ws/tools/joint_panel/index.html) : Ajout d'une section dédiée Chifoumi dans l'interface web de pilotage avec boutons de commande directe des poses et déclenchement de la séquence d'animation.

- **Documentation** :
  - [`docs/CHIFOUMI.md`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/docs/CHIFOUMI.md) : Guide exhaustif incluant une section de vulgarisation scientifique pour les personnes non-roboticiennes (vision par ordinateur, servomoteurs, synchronisation temps réel, arbitrage), la matrice de poses et le guide d'utilisation pas-à-pas.
  - [`docs/JOURNAL.md`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/docs/JOURNAL.md) : Ajout de l'étape 24 documentant la conception et l'intégration du projet Chifoumi.

### Corrigé
- [`ros2_ws/src/so101_perception/so101_perception/perception_node.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/ros2_ws/src/so101_perception/so101_perception/perception_node.py#L233) : Correction d'une faute de frappe syntaxique (`8,`) présente à la ligne 233 après `self._detect_bowl(image, debug)`.
- [`scripts/chifoumi.sh`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/scripts/chifoumi.sh) et [`handGesture/chifoumi_game.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/chifoumi_game.py) : Résolution automatique de la racine du dépôt (`sys.path` et `PYTHONPATH`) pour permettre l'exécution directe depuis n'importe quel sous-dossier (ex. `scripts/`).
- [`handGesture/chifoumi_game.py`](file:///Users/techeretienne/Documents/claude_workspace/Robotics/content-lerobot-bras-students/handGesture/chifoumi_game.py) : Maintien prolongé de la pose (Pierre, Feuille ou Ciseaux) pendant 5 à 10 secondes (défaut 6.0s, réglable via `--hold`) dans RViz/simulation sans interruption prématurée, avec retour automatique et fluide en garde (`READY`) avant la manche suivante.
