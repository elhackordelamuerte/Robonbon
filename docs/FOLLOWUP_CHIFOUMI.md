# 🎤 Follow-up : script oral Chifoumi (~3 min)

Points vérifiés par l'encadrant :
1. Communication ROS 2 **entre conteneurs**
2. `driver_node` qui publie **`/joint_states`**
3. Le bras qui bouge via **`/joint_command`**
4. Aide sur la **calibration caméra** si besoin

> Les blocs « 🗣️ » sont à dire, les blocs « 📂 » sont les fichiers ou lignes à montrer à l'écran,
> et les commandes « ▶ N » sont à taper **dans l'ordre**, dans le terminal indiqué.

---

## 🖥️ Les 3 terminaux

| Terminal | Où | Rôle |
|---|---|---|
| **T1** | Mac (racine du dépôt) | Lancer la stack, puis le jeu |
| **T2** | Shell dans le conteneur **`perception`** | Toutes les commandes `ros2 …` (preuve inter-conteneurs) |
| **T3** | Mac (racine du dépôt) | Commandes `docker compose` ponctuelles + calibration |

Navigateur : `http://localhost:6080` (RViz) et `http://localhost:8080` (panneau).

---

## ✅ Avant la séance (préparation, ~3 min)

```bash
# ▶ 1 [T1] Lancer toute la stack (control + viz + perception, panneau, RViz)
./scripts/start.sh
#    attendre « ✅ Driver prêt », « ✅ Panneau prêt », « ✅ RViz prêt »

# ▶ 2 [T2] Ouvrir un shell dans perception et charger ROS 2
#    (une seule commande : ROS est chargé PUIS le shell interactif s'ouvre)
docker compose -f docker/docker-compose.yml exec perception bash -c "source /opt/ros/jazzy/setup.bash && exec bash"
#    vérif : ros2 --help doit répondre

# ▶ 3 [T1] Vérifier que la webcam reconnaît la main (q pour quitter)
./scripts/chifoumi.sh --test
```

- [ ] Onglets RViz (6080) et panneau (8080) ouverts
- [ ] Fichiers ouverts dans l'éditeur, dans l'ordre de passage (📂 de chaque section)
- [ ] `ros2_ws/calib/` présent sur la machine (ignoré par git)

> ⚠️ `docker compose exec` ouvre un shell **sans** ROS : sans le `source` de l'étape ▶ 2,
> on obtient `ros2: command not found`.

---

## 1. Contexte (20 s)

🗣️ « Une fois le pick & place en place, on a ajouté un mode interactif : un Pierre-Feuille-Ciseaux
contre le SO-ARM101. La webcam lit le geste de la main, et le bras joue son coup en même temps.
Ce mode réutilise la même stack ROS 2 que le reste du projet, donc il nous sert aussi à valider
la chaîne de communication de bout en bout. »

📂 `docs/CHIFOUMI.md` section 2 (schéma d'architecture).

*(pas de commande)*

---

## 2. Communication ROS 2 entre conteneurs (40 s)

🗣️ « La stack tourne dans Docker Compose : un conteneur `control`, un conteneur `perception`
et un conteneur `viz`. Ils sont tous sur un même réseau Docker, `rosnet`, avec un
`ROS_DOMAIN_ID` commun (42). La découverte DDS se fait donc toute seule entre conteneurs,
sans `--network host`. Le `ROS_DOMAIN_ID` isole aussi notre groupe des autres équipes
sur la même machine. »

📂 `docker/docker-compose.yml`
- **l.14-17** : bloc `x-ros-env` partagé (`ROS_DOMAIN_ID`, transport FastDDS)
- **l.20 / l.56 / l.82** : services `control`, `perception`, `viz`
- **l.48, 65, 88** : `networks: [rosnet]` sur chaque service
- **l.95-96** : déclaration du réseau `rosnet`

```bash
# ▶ 4 [T3] Montrer les 3 conteneurs qui tournent
docker compose -f docker/docker-compose.yml ps

# ▶ 5 [T2] Depuis perception, on voit les nœuds de control et viz
ros2 node list          # 1er appel parfois incomplet : attendre 3-4 s et relancer (PAS --no-daemon)
#    attendu : /so101_driver, /so101_brain, /joint_panel, /robot_state_publisher, /rviz, /so101_perception

# ▶ 6 [T2] … et leurs topics
ros2 topic list
```
📋 Sortie de ▶ 5 : qui tourne où

| Nœud | Conteneur | Rôle |
|---|---|---|
| `/so101_driver` | `control` | Notre `driver_node` : `/joint_command` → simu → `/joint_states` |
| `/so101_brain` | `control` | Pick & place (services de grab) |
| `/joint_panel` | `tek5-panel` | Panneau web : API REST → `/joint_command` |
| `/robot_state_publisher` | `viz` | `/joint_states` → arbre TF (`/tf`) |
| `/drop_box_marker_pub` | `viz` | Marqueur de la boîte de dépôt dans RViz |
| `/static_transform_publisher_…` | `viz` | TF fixe lancé par `viz.launch.py` |
| `/rviz` | `tek5-rviz` | Visualisation 3D |
| `/so101_perception` | `perception` | Le seul nœud local au conteneur où l'on tape la commande |
| `/transform_listener_impl_…` | probablement `rviz` | Nœud interne créé automatiquement par un listener TF2 (pas de code à nous) |

👉 À dire : « seul `/so101_perception` tourne dans ce conteneur ; tous les autres sont découverts
par DDS à travers le réseau `rosnet`. »

👉 À dire : « on est dans `perception`, et pourtant on voit `/so101_driver` (notre `driver_node`) et `/joint_states`,
qui vivent dans `control`. »

---

## 3. `driver_node` → `/joint_states` (30 s)

🗣️ « Le `driver_node` sert de pont entre ROS et le bras. Il reçoit les consignes sur
`/joint_command`, les envoie au backend (MuJoCo en simulation, ou le bus série pour le bras
réel) et publie l'état réel des articulations sur `/joint_states` à 30 Hz. Sa boucle de contrôle
tourne à 50 Hz. »

📂 `ros2_ws/src/so101_driver/so101_driver/driver_node.py`
- **l.47-48** : `CONTROL_HZ = 50.0`, `JOINT_STATE_HZ = 30.0`
- **l.88-91** : publisher `/joint_states`, subscriber `/joint_command`, les deux timers (**le cœur du nœud, à montrer en priorité**)
- **l.113** : `_connect_arm()`, choix sim (`SO101Sim`) ou réel (`SO101Follower`)
- **l.153** : `_cb_joint_command()`, validation des noms et tailles (joint inconnu → warning)
- **l.163** : `_rate_limited_goal()`, limitation de vitesse (sécurité des servos)
- **l.178** : `_publish_joint_states()`

```bash
# ▶ 7 [T2] Qui publie / qui s'abonne
ros2 topic info /joint_states -v

# ▶ 8 [T2] Fréquence de publication (≈ 30 Hz), puis Ctrl+C
ros2 topic hz /joint_states

# ▶ 9 [T2] Un message : noms des 6 joints + positions
ros2 topic echo /joint_states --once
```

---

## 4. Le bras bouge via `/joint_command` (50 s)

🗣️ « Comme le demande le sujet, tous les mouvements passent par `/joint_command`.
Côté Chifoumi, la vision tourne sur le Mac hôte, parce que Docker Desktop ne donne pas accès à la
webcam USB. MediaPipe détecte 21 points de la main, puis on classe le geste et on le lisse par
vote majoritaire. Le jeu envoie ensuite une pose (PIERRE, FEUILLE ou CISEAUX) à notre panneau
web. Ce panneau est un nœud ROS : il interpole la trajectoire et la publie sur `/joint_command`
à 30 Hz. Le compte « 1, 2, 3… » est synchronisé avec le bras. Au « shoot », le robot montre
son coup, tiré au hasard, au moment même où l'image de la main est figée : aucun des deux joueurs
ne peut tricher. »

```bash
# ▶ 10 [T2] Commande manuelle → regarder RViz : le bras pivote
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan], position: [0.5]}"

# ▶ 11 [T2] Le ramener au centre
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan], position: [0.0]}"

# ▶ 12 [T2] Écouter les consignes AVANT de lancer le jeu (laisser tourner)
ros2 topic echo /joint_command

# ▶ 13 [T1] Lancer le jeu sur le Mac, puis ESPACE pour jouer une manche
./scripts/chifoumi.sh
#    → T2 défile pendant le compte 1-2-3 et le coup du robot

# ▶ 14 Fin de démo : Q dans la fenêtre du jeu, Ctrl+C dans T2
```

📂 Pendant que le jeu tourne, la chaîne complète de la main au topic :

| Étape | Fichier | Lignes à montrer |
|---|---|---|
| Détection doigts | `handGesture/hand_detector.py` | **l.67** `detecter_doigts_leves()` |
| Classification | `handGesture/hand_detector.py` | **l.108** `classifier_geste()` |
| Lissage | `handGesture/hand_detector.py` | **l.64-65** (deque) + **l.157** vote majoritaire |
| Coup du robot / arbitre | `handGesture/chifoumi_game.py` | **l.127-130** `evaluer_manche()` + `random.choice` ; **l.70** `determiner_vainqueur()` |
| Poses articulaires | `handGesture/robot_chifoumi.py` | dictionnaire `POSES_CHIFOUMI` (haut du fichier) |
| Envoi HTTP | `handGesture/robot_chifoumi.py` | **l.110** `envoyer_pose()` → `POST /api/commande` |
| Nœud ROS du panneau | `ros2_ws/tools/joint_panel/panel.py` | **l.84-85** publisher `/joint_command` + abonnement `/joint_states` |
| Publication | `ros2_ws/tools/joint_panel/panel.py` | **l.168** `publier()`, **l.194** `_jouer()` (interpolation à 30 Hz, l.34) |

👉 Phrase clé : « le jeu ne parle jamais aux moteurs directement, tout passe par `/joint_command` »
(cf. commentaire l.12 de `panel.py`).

---

## 5. Calibration caméra : où on en est (30 s)

🗣️ « Pour la calibration, on a généré nos cibles : un damier 9×6 de 25 mm et un marqueur
ArUco de 80 mm. La pose de la caméra dans le repère robot a été calculée avec l'ArUco et
`solvePnP`, elle est dans `camera_pose.yaml`. Les intrinsèques, en revanche, sont encore
une **approximation** (champ de vision supposé de 75°, distorsion nulle). Le script
`calibrate_intrinsics.py` est prêt, mais pas encore lancé sur la vraie caméra.
**C'est là qu'on aimerait votre aide** : valider la procédure du damier et la précision
de la pose obtenue. »

📂 À montrer
- `ros2_ws/calib/camera.yaml` : champ `approximation` (preuve honnête que K n'est pas encore calibré)
- `ros2_ws/calib/camera_pose.jpg` : image de la pose ArUco détectée
- `scripts/calibrate_intrinsics.py` **l.56** `findChessboardCorners`, **l.122-124** `calibrateCamera` → `ros2_ws/calib/camera.yaml`
- `scripts/calibrate_pose.py` **l.79** `detectMarkers`, **l.83** `solvePnP` → `ros2_ws/calib/camera_pose.yaml`
- `scripts/generate_targets.py` : génération des cibles (`calib_targets/*.pdf`)
- `sim/so101_sim/__init__.py` **l.149 / l.157** : `get_camera_extrinsics()` / `get_camera_intrinsics()` (valeurs de la simu)

```bash
# ▶ 15 [T3] Montrer l'état actuel de la calibration
cat ros2_ws/calib/camera.yaml
open ros2_ws/calib/camera_pose.jpg
```

Seulement si on calibre en séance avec l'encadrant :
```bash
# ▶ 16 [T3] Diffuser la webcam de l'hôte vers les conteneurs (laisser tourner)
uv run scripts/camera_stream.py --list         # repérer le nom de la caméra
uv run scripts/camera_stream.py --nom UGREEN   # recommandé sur macOS (choix par nom)

# ▶ 17 [nouveau terminal, Mac] Intrinsèques au damier (case mesurée au réglet)
uv run scripts/calibrate_intrinsics.py --cible damier --case-mm 25

# ▶ 18 [même terminal] Pose caméra avec l'ArUco posé à plat à une position connue
uv run scripts/calibrate_pose.py --x 0.25 --y 0.0 --marqueur-mm 80
```
⚠️ Imprimer à **100 %** et **mesurer au réglet** la taille réelle d'une case et du marqueur.

❓ Questions à poser à l'encadrant :
- Combien d'images de damier, et quelle erreur RMS viser ?
- Comment vérifier la pose `cam_T` (par exemple viser un point connu avec le bras) ?

---

## Conclusion (10 s)

🗣️ « La communication entre conteneurs marche, le driver publie `/joint_states`, et le bras
se commande uniquement par `/joint_command`, en simulation comme sur le bras réel.
Il nous reste à finir la calibration intrinsèque. »

```bash
# ▶ 19 [T1] Après la séance : tout arrêter
./scripts/stop.sh
```

---

## 📋 Antisèche : ordre des commandes

| # | Terminal | Commande |
|---|---|---|
| 1 | T1 | `./scripts/start.sh` |
| 2 | T2 | `docker compose -f docker/docker-compose.yml exec perception bash -c "source /opt/ros/jazzy/setup.bash && exec bash"` |
| 3 | T1 | `./scripts/chifoumi.sh --test` |
| 4 | T3 | `docker compose -f docker/docker-compose.yml ps` |
| 5 | T2 | `ros2 node list` |
| 6 | T2 | `ros2 topic list` |
| 7 | T2 | `ros2 topic info /joint_states -v` |
| 8 | T2 | `ros2 topic hz /joint_states` (Ctrl+C) |
| 9 | T2 | `ros2 topic echo /joint_states --once` |
| 10 | T2 | `ros2 topic pub --once /joint_command … position: [0.5]` |
| 11 | T2 | `ros2 topic pub --once /joint_command … position: [0.0]` |
| 12 | T2 | `ros2 topic echo /joint_command` (laisser tourner) |
| 13 | T1 | `./scripts/chifoumi.sh` puis ESPACE |
| 14 | — | Q dans le jeu, Ctrl+C dans T2 |
| 15 | T3 | `cat ros2_ws/calib/camera.yaml` · `open ros2_ws/calib/camera_pose.jpg` |
| 16-18 | T3 + Mac | Calibration en séance (optionnelle) |
| 19 | T1 | `./scripts/stop.sh` |

> ℹ️ Les numéros de ligne correspondent à l'état de la branche `feat/grab-camera` au 2026-10-21.
