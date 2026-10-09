# Perception et commande du bras : ce qui se passe, topic par topic

Ce document suit **une image** depuis la caméra jusqu'au mouvement du bras. Il précise, pour chaque étape,
**quel nœud** agit, **quels topics/services ROS 2** sont utilisés, et **comment l'observer** soi-même.
Pour la théorie (sténopé, IK, machine d'états), voir [GRAB_CAMERA.md](GRAB_CAMERA.md).

---

## 1. Le parcours en un coup d'œil

```
 CAMÉRA                PERCEPTION                    BRAIN                       DRIVER             ROBOT
 ──────                ──────────                    ─────                       ──────             ─────
 image ──► /…/image_raw ─┐
 K     ──► /…/camera_info┤
                         ▼
               perception_node
               HSV → contour → (u,v)
               rayon K⁻¹ → plan z
                         │
                         ├──► /object_position ────┐
                         ├──► /drop_box_position ──┤
                         └──► /perception/debug    ▼
                                             brain_node  ◄── /pick_and_place (service)
                                             IK + 15 états
                                                   │
                                                   └──► /joint_command ──► driver_node ──► send_action()
                                                                                │
                                            ◄────────── /joint_states ◄─────────┘ get_observation()
```

**Boucle fermée :** le brain lit `/joint_states` (position réelle) pour partir de l'état mesuré, et la perception
lui fournit en continu la dernière position connue. Le brain n'attend jamais la perception : il utilise la dernière valeur reçue.

---

## 2. Les deux sources de caméra

| | Caméra **simulée** (MuJoCo) | Caméra **réelle** (webcam USB) |
|---|---|---|
| Qui publie l'image | `driver_node` (`render_camera()`) | `camera_node` (flux OpenCV) |
| Topics | `/external_cam/image_raw`, `/external_cam/camera_info` | `/camera/image_raw`, `/camera/camera_info` |
| Matrice K | exacte (`get_camera_intrinsics()`) | approximée (`hfov_deg`) tant qu'il n'y a pas de calibration au damier |
| Pose caméra `cam_T` | paramètre `cam_T` du driver (vérité terrain, autorisée pour démarrer) | **à calibrer** (ArUco + `solvePnP`) → paramètre `cam_T` de `perception_node` |
| Résultat | positions 3D → grab complet | **mode 2D** : détection + image annotée, pas encore de position 3D |
| Lancement | `CAMERA=sim` (défaut) | `CAMERA=real` |

Les deux sources utilisent **les mêmes types de messages**. `perception_node` choisit son topic avec le paramètre
`camera_ns` (`/external_cam` ou `/camera`). Les topics sont distincts pour qu'une caméra réelle puisse tourner pendant
que la sim publie la sienne, sans conflit.

### Pourquoi un script sur le Mac ?

Docker Desktop (macOS) **ne donne pas accès aux webcams** depuis les conteneurs (pas de `/dev/video0`). D'où :

```
webcam USB ──AVFoundation──► scripts/camera_stream.py (Mac, port 8081, MJPEG)
                                     │ http://host.docker.internal:8081/stream.mjpg
                                     ▼
                            camera_node (conteneur perception) ──► /camera/image_raw
```

Sous Linux, on peut se passer du script : monter `/dev/video0` dans le conteneur et passer `CAMERA_SOURCE=0`.

---

## 3. Référence des topics

| Topic | Type | Publié par | Lu par | Fréquence | Contenu |
|---|---|---|---|---|---|
| `/external_cam/image_raw` | `sensor_msgs/Image` | driver (sim) | perception | 10 Hz | image RGB 320×240 |
| `/external_cam/camera_info` | `sensor_msgs/CameraInfo` | driver (sim) | perception | 10 Hz | K (fx=fy≈246, cx=160, cy=120) |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node | perception | 10 Hz | image RGB de la webcam (640×480) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_node | perception | 10 Hz | K (calibrée ou approximée) |
| `/object_position` | `geometry_msgs/PoseStamped` | perception (ou driver si `fake_perception`) | brain, panneau | ~10 Hz | centre du cube (m, repère `world`) + yaw dans l'orientation |
| `/drop_box_position` | `geometry_msgs/PoseStamped` | perception (ou driver si `fake_perception`) | brain, panneau | ~10 Hz | centre de la boîte (m, `world`) |
| `/perception/debug` | `sensor_msgs/Image` | perception | RViz (display *Image*) | 10 Hz | image annotée : contour vert = cube retenu, orange = cube coupé par le bord (ignoré), bleu = boîte |
| `/joint_states` | `sensor_msgs/JointState` | driver | brain, robot_state_publisher, panneau | 30 Hz | angles mesurés (rad, pince en rad) |
| `/joint_command` | `sensor_msgs/JointState` | brain, panneau | driver | 50 Hz pendant un mouvement | consignes (rad, **pince en %**) |
| `/end_effector_pose` | `geometry_msgs/PoseStamped` | brain | RViz | 10 Hz | pose du TCP calculée par FK |
| `/rosout` | `rcl_interfaces/Log` | tous | panneau (journal du brain) | — | logs, dont les étapes `pick & place: …` |

**Repère :** `world` = `base_link` (TF statique identité, publiée par `viz`). x vers l'avant du robot, z vers le haut.

## 4. Services et paramètres

| Service | Type | Fourni par | Effet |
|---|---|---|---|
| `/pick_and_place` | `std_srvs/Trigger` | brain | planifie (IK) puis exécute la séquence avec les **dernières** positions reçues |
| `/go_to_rest` | `std_srvs/Trigger` | brain | position de repos `(0.18, 0.10, 0.17)` |
| `/go_to_target` | `so101_interfaces/GoToTarget` | brain | amène le TCP à un point XYZ |
| `/set_joint_positions` | `so101_interfaces/SetJointPositions` | brain | 6 joints d'un coup (degrés, pince %) |
| `/reset_object` | `std_srvs/Trigger` | driver (sim) | replace le cube au hasard ; la réponse donne la vraie position (utile pour mesurer l'erreur) |

| Paramètre | Nœud | Rôle |
|---|---|---|
| `fake_perception` | driver | `true` : publie la vérité terrain sur `/object_position` (**interdit** en évaluation, et en conflit avec la perception) |
| `cam_T` | driver (sim) | pose caméra→monde 4×4 de la sim, lue par la perception si elle n'a pas la sienne |
| `camera_ns` | perception | `/external_cam` (sim) ou `/camera` (réel) |
| `cam_T` | perception | pose caméra→monde issue de **votre** calibration (16 valeurs, ligne par ligne) |
| `cam_T_depuis_driver` | perception | `false` en réel : ne pas utiliser la pose de la caméra virtuelle |
| `convention_mujoco` | perception | `true` en sim (axe optique −Z) ; `false` pour un `cam_T` issu de `solvePnP` (convention OpenCV) |
| `hsv_rouge` | perception | 12 entiers = 2 plages `[h,s,v min, h,s,v max]`, à régler sous votre éclairage |
| `aire_min_cube_px` | perception | aire minimale du contour (40 en sim, 400 en réel) contre les fausses détections |
| `source`, `calibration_file`, `hfov_deg`, `fps` | camera_node | source OpenCV, YAML de calibration, champ supposé, cadence |

---

## 5. Étape par étape : ce qui se passe pendant un grab

1. **Image.** Le driver (sim) ou `camera_node` (réel) publie une image et sa `camera_info` 10 fois par seconde.
2. **Détection 2D** (`perception_node._detect_object_2d`) : conversion RGB→HSV, masque rouge (deux plages autour de
   0° et 180°, `V ≥ 100` pour exclure le marron de la boîte), fermeture morphologique, plus grand contour, centre `(u, v)`
   par les moments. Un contour qui touche le bord de l'image est **ignoré** (cube coupé = centre faux).
3. **Passage en 3D** (`_project_to_3d`) : rayon `K⁻¹·[u, v, 1]`, conversion d'axes, rotation par `cam_T`, intersection
   avec le plan **z = 2,75 cm** (hypothèse : cube posé sur la table). Le yaw vient du contour projeté sur le dessus
   du cube (`minAreaRect`).
4. **Boîte** : même principe en marron. Elle est fixe et souvent masquée par le bras, donc on garde l'observation de **plus
   grande aire**, projetée à z = 3,6 cm (haut des parois).
5. **Publication** de `/object_position` et `/drop_box_position` (`PoseStamped`, `frame_id: world`).
6. **Déclenchement** : `/pick_and_place` (panneau ou CLI). Le brain copie la **dernière** position reçue, puis planifie
   dans un thread : choix de la face du cube, IK verticale, trajectoires en ligne droite tous les 1 cm.
7. **Exécution** : un timer à 50 Hz publie `/joint_command` (interpolation en cosinus) pour chacun des 15 états.
8. **Driver** : `send_action()` puis `get_observation()`. En sim, MuJoCo simule les contacts : la pince serre vraiment le cube.
9. **Retour** : `/joint_states` → le brain connaît la position réelle, `robot_state_publisher` met à jour les TF, RViz affiche.

**Mesures en sim (perception seule, `fake_perception = false`) :** erreur de position du cube 1 à 2 mm, boîte environ 1,2 cm,
**4 grabs réussis sur 4** quand le cube est visible. Dans environ 1 tirage sur 5, le cube sort du champ de la caméra, et le
grab n'est alors pas lancé.

---

## 6. Lancer et observer

### Simulation (caméra MuJoCo)

```bash
./scripts/start.sh                       # control + viz + perception + panneau + RViz
```
Dans le panneau (http://localhost:8080), carte *Grab d'objet* : décocher *Perception simulée*, puis *Nouveau cube* → *Pick & place*.

### Caméra réelle (webcam USB, mode 2D)

```bash
uv run scripts/camera_stream.py --list            # noms des caméras
uv run scripts/camera_stream.py --nom UGREEN      # par NOM (ffmpeg) ; aperçu : http://localhost:8081
CAMERA=real docker compose -f docker/docker-compose.yml up -d --force-recreate perception
```

> ⚠️ Les **index** de caméra changent sur macOS (ex. l'iPhone en caméra Continuité s'insère dans la liste) et ne sont pas
> dans le même ordre pour OpenCV et ffmpeg. Utiliser **`--nom UGREEN`** (nécessite `ffmpeg`, `brew install ffmpeg`).

### Commandes d'observation (dans `docker compose exec control bash`, après les `source`)

```bash
ros2 topic hz /camera/image_raw                          # la caméra arrive-t-elle ?
ros2 topic echo /object_position --field pose.position   # position estimée du cube
ros2 topic info -v /object_position                      # qui publie ? (perception ET driver = fake_perception actif ?)
ros2 param get /so101_perception camera_ns
ros2 param set /so101_perception hsv_rouge "[0,120,100,12,255,255,160,120,100,180,255,255]"
ros2 service call /pick_and_place std_srvs/srv/Trigger
```

Dans RViz : *Add* → *By topic* → `/perception/debug` → *Image*, pour voir ce que la perception détecte.

---

## 7. Pour passer au grab réel

1. **Calibration intrinsèque** (damier imprimé, ~15 vues) : `cv2.calibrateCamera` → YAML (`camera_matrix`, `dist_coeffs`)
   → `CAMERA_CALIBRATION=/ros2_ws/…/camera.yaml`.
2. **Pose caméra** : marqueur ArUco posé à une position connue dans le repère du robot → `cv2.solvePnP` → inverser pour obtenir
   caméra→monde → paramètre `cam_T` de `perception_node` (`convention_mujoco: false`).
3. **Seuils HSV** sous l'éclairage réel (`hsv_rouge`), contrôlés sur `/perception/debug`.
4. **Bras réel** : `USE_SIM=false` (driver), le brain et la perception restent identiques.
