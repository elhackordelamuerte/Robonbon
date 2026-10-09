# Attraper un objet vu par une caméra avec ROS 2

Ce document explique **comment le SO-ARM101 attrape le cube rouge** : quels nœuds ROS 2 sont impliqués,
quelles données circulent, et quels procédés (vision, géométrie, cinématique, commande) sont utilisés
à chaque étape. Il décrit le code actuel (branche d'Arthur) et ce qu'il reste à faire pour la caméra.

> **État actuel :** `perception_node` (OpenCV) est implémenté. En sim, le grab fonctionne **uniquement avec la caméra**
> (4 réussites sur 4 quand le cube est visible). Une caméra réelle (webcam USB) est branchée en mode 2D,
> en attendant la calibration. Détail des nœuds, des topics et des commandes : **[PERCEPTION_ROS.md](PERCEPTION_ROS.md)**.

---

## 1. Vue d'ensemble

```
             ┌───────────────────────── /external_cam/image_raw (10 Hz, rgb8) ──────────┐
             │                          /external_cam/camera_info (matrice K)           ▼
┌────────────┴───┐                                                           ┌──────────────────┐
│  driver_node   │                                                           │ perception_node  │
│ (sim / réel)   │                                                           │      OpenCV      │
└──▲─────────┬───┘                                                           └────────┬─────────┘
   │         │ /joint_states (30 Hz)                                                  │ /object_position
   │         │                                                                        │ /drop_box_position
   │         ▼                                                                        ▼
   │   ┌───────────────────────────────────────────────────────────────────────────────────┐
   │   │ brain_node : cinématique (FK/IK) + machine d'états du pick & place                │
   │   └──────────────────────────────────────┬────────────────────────────────────────────┘
   └──────────── /joint_command (50 Hz) ──────┘
```

| Nœud | Rôle | Conteneur |
|---|---|---|
| `driver_node` | Seul à parler au robot (`SO101Sim` ou `SO101Follower`). Publie l'état des joints et l'image caméra, exécute `/joint_command`. | `control` |
| `perception_node` | Trouve le cube et la boîte dans l'image, publie leurs positions 3D dans le repère robot. | `perception` |
| `brain_node` | Calcule les trajectoires (IK) et enchaîne les étapes du pick & place. | `control` |
| `robot_state_publisher` | Transforme `/joint_states` en TF (position de chaque link) pour RViz. | `viz` |

**Règle du sujet :** seul le driver accède au backend. Brain et perception communiquent **uniquement par topics**.

---

## 2. Les mécanismes ROS 2 utilisés

| Mécanisme | Utilisation ici | Pourquoi |
|---|---|---|
| **Topic** (publish/subscribe) | `/joint_states`, `/joint_command`, `/external_cam/image_raw`, `/object_position` | Flux continus : le dernier message suffit, et personne n'attend de réponse. |
| **Service** (requête/réponse) | `/pick_and_place`, `/go_to_rest`, `/reset_object` (`std_srvs/Trigger`), `/go_to_target`, `/set_joint_positions` | Actions ponctuelles, avec un succès ou un échec à renvoyer. |
| **Paramètre** | `use_sim`, `fake_perception` (driver), `travel_z`, `grasp_z`, `gripper_open`… (brain) | Réglages modifiables sans recompiler : `ros2 param set /so101_driver fake_perception true`. |
| **Timer** | Contrôle à 50 Hz, état à 30 Hz, caméra à 10 Hz, pose de l'outil à 10 Hz | Boucles périodiques non bloquantes, dans l'executor du nœud. |
| **TF2** | `world → base_link` (statique), puis un TF par link via `robot_state_publisher` | Positions relatives entre repères, affichées par RViz. |
| **cv_bridge** | Image MuJoCo (numpy) ↔ `sensor_msgs/Image` | Conversion standard entre OpenCV et ROS. |
| **DDS / `ROS_DOMAIN_ID`** | Découverte automatique entre conteneurs du réseau `tek5_rosnet` | Aucune adresse à configurer : même domaine = mêmes topics. |

Messages principaux :

- `sensor_msgs/JointState` : `name[]` et `position[]`. Radians pour les joints. La pince est en **%** sur `/joint_command`,
  mais en **radians** sur `/joint_states`, pour que `robot_state_publisher` puisse placer le link.
- `sensor_msgs/Image` et `sensor_msgs/CameraInfo` : l'image et sa matrice intrinsèque `K`.
- `geometry_msgs/PoseStamped` : position (et orientation) d'un objet, avec son repère dans `header.frame_id`.

---

## 3. La caméra

Dans la scène MuJoCo (`scene_tek5.xml`), la caméra `external_cam` est **fixe**, en `(0.45, 0.45, 0.55)` m,
orientée vers le robot, avec un champ vertical `fovy = 52°`. Elle produit des images de 320×240.

Le driver publie à 10 Hz :
- `/external_cam/image_raw` : `render_camera()` converti par `cv_bridge` (`rgb8`) ;
- `/external_cam/camera_info` : la matrice **K** issue de `get_camera_intrinsics()`.

**Modèle sténopé (pinhole)** : un point 3D `(X, Y, Z)` exprimé dans le repère caméra se projette sur le pixel `(u, v)` :

```
    ⎡u⎤   1 ⎡fx  0  cx⎤ ⎡X⎤          fy = (H/2) / tan(fovy/2)  ≈ 246 px
    ⎢v⎥ = ─ ⎢ 0 fy  cy⎥ ⎢Y⎥          fx = fy (pixels carrés)
    ⎣1⎦   Z ⎣ 0  0   1⎦ ⎣Z⎦          (cx, cy) = (W/2, H/2) = (160, 120)
```

On le voit dans RViz grâce au display *Camera* de `so101.rviz` (topic `/external_cam/image_raw`).

---

## 4. Perception : de l'image à la position 3D

Implémenté dans `so101_perception/perception_node.py` (valeurs mesurées : voir [PERCEPTION_ROS.md](PERCEPTION_ROS.md)). Le procédé :

### 4.1 Détecter le cube rouge (2D)

1. Convertir l'image en **HSV** (`cv2.cvtColor(img, cv2.COLOR_RGB2HSV)`). Contrairement au RGB, la teinte (H)
   varie peu avec l'éclairage.
2. **Seuiller le rouge**. Le rouge est à cheval sur 0° et 180° en HSV, donc il faut deux plages réunies :
   `cv2.inRange(hsv, (0,120,70), (10,255,255)) | cv2.inRange(hsv, (170,120,70), (180,255,255))`.
3. **Nettoyer** le masque par ouverture/fermeture morphologique (`cv2.morphologyEx`), pour enlever le bruit
   et boucher les trous (le cube est percé).
4. **Contours** (`cv2.findContours`) : garder le plus grand, au-dessus d'une aire minimale.
5. **Centre** grâce aux moments : `u = M10/M00`, `v = M01/M00`.
   L'orientation (yaw du cube) s'obtient avec `cv2.minAreaRect`.

La boîte de dépôt se détecte de la même façon, avec sa propre couleur.

> D'autres méthodes sont possibles (YOLO, marqueurs ArUco…), mais le sujet demande de **justifier le choix**.
> Pour un cube unicolore sur fond neutre, HSV + contours est simple, rapide et suffisant.

### 4.2 Passer du pixel au point 3D

Un pixel ne donne qu'une **direction** : tous les points sur ce rayon se projettent au même pixel.
Il faut donc une **hypothèse** supplémentaire. Ici, *le cube est posé sur la table*, donc son centre est à
`z = 0.025` m (demi-côté du cube de 5 cm).

1. **Rayon dans le repère caméra** : `d_cam = K⁻¹ · [u, v, 1]ᵀ`.
2. **Changement de convention** : MuJoCo place l'axe optique sur `-Z` avec `Y` vers le haut, alors qu'OpenCV
   utilise `+Z` devant et `Y` vers le bas. On corrige avec `d_mj = diag(1, -1, -1) · d_cam`.
3. **Rayon dans le repère monde**, avec la pose caméra→monde `T = [R | t]` (extrinsèques) :
   `d_w = R · d_mj`, origine `o = t`.
4. **Intersection avec le plan** `z = z_cube` : `s = (z_cube − o_z) / d_w,z`, puis `P = o + s · d_w`.
5. Publier `P` sur `/object_position` (`PoseStamped`, `frame_id = "world"`) à 5-10 Hz.
   Ici, `world` et `base_link` coïncident (TF statique identité).

```
  caméra o ●
            ╲  rayon d_w (pixel du centre du cube)
             ╲
              ╲
   ────────────●─────────── plan z = 0.025 m (centre du cube posé)
               P = o + s·d_w
```

### 4.3 Calibration : d'où viennent K et T ?

| Donnée | En simulation | Sur le robot réel |
|---|---|---|
| **K** (intrinsèques) | `get_camera_intrinsics()` → `/camera_info` | Calibration au damier (`cv2.calibrateCamera`) |
| **T** (extrinsèques, pose caméra) | `get_camera_extrinsics()` : **autorisé pour démarrer**, mais il faut à terme sa propre calibration | Mesure obligatoire : ArUco/damier posé à un endroit connu, puis `cv2.solvePnP` |

`get_object_position()` est **interdit** dans le pipeline : il ne sert qu'à **mesurer l'erreur** de la perception.

---

## 5. Brain : cinématique

Fichier : `so101_brain/kinematics.py` (classe `Arm`, construite à partir du URDF).

- **Cinématique directe (FK)** : `fk(q)` → pose 4×4 du TCP (`gripper_frame_link`) dans `base_link`, calculée
  avec **ikpy** à partir de la chaîne du URDF. Elle est publiée sur `/end_effector_pose`.
- **Cinématique inverse (IK)** pour une prise par le dessus, `ik_grasp(cible, yaw)` : **moindres carrés pondérés**
  (`scipy.optimize.least_squares`) bornés aux limites des joints. Trois termes, par ordre de priorité :

  | Terme | Poids | Rôle |
  |---|---|---|
  | position du TCP − cible | 1.0 | atteindre le point |
  | axe X du TCP − direction des mâchoires (yaw) | 0.01 | aligner la pince avec une face du cube |
  | axe Z du TCP − verticale (0,0,−1) | 0.003 | descendre bien à la verticale |

  Le solveur part de **plusieurs graines** (la configuration actuelle, zéro, puis des tirages aléatoires) pour
  éviter les minima locaux. Pour les trajectoires, il raffine seulement depuis le point précédent
  (`n_random=-1`), ce qui évite les sauts de configuration.

  L'IK d'ikpy seule (avec orientation) manquait la position de 5 à 8 mm près des limites du poignet :
  c'est la raison de ce solveur maison.

---

## 6. Brain : planification et machine d'états

Fichier : `so101_brain/brain_node.py`. Le service `/pick_and_place` lit les **dernières** positions reçues
de l'objet et de la boîte : le contrôle n'attend jamais la perception. La planification (quelques secondes d'IK)
tourne dans un **thread**, puis un timer à 50 Hz exécute les étapes.

**Choix de la prise :**
- le cube est invariant par rotation de 90°, donc les **4 faces** sont testées. Celle qui demande le moins
  de rotation du poignet (`wrist_roll`) est retenue ;
- la mâchoire mobile vient se loger dans le **trou du dessus** du cube, la mâchoire fixe longe une face.
  Le TCP est donc décalé de `(tip_x, tip_y)` par rapport au centre du cube ;
- le brain cherche le point le plus haut où la pince est encore parfaitement verticale (tilt < 0,5°). À partir
  de là, la descente est une translation purement verticale.

**Trajectoires en ligne droite :** chaque déplacement est découpé en points espacés de **1 cm**, avec une IK
par point (`_line`). L'interpolation dans le temps utilise une courbe en cosinus (`_ease`), qui démarre et
s'arrête en douceur.

**Séquence** (chaque ligne = un état, commandes publiées sur `/joint_command`) :

| # | État | Action | Durée |
|---|---|---|---|
| 1 | move above object | ligne droite vers `travel_z` = 15 cm au-dessus du cube | 3,5 s |
| 2 | approach | descente jusqu'au pré-grasp vertical (6-10 cm) | 2 s |
| 3 | open gripper | ouverture à 16 % | 1 s |
| 4 | align | pause pour stabiliser | 0,5 s |
| 5 | descend | descente verticale jusqu'à `seat_z` = 3,2 cm | 2,5 s |
| 6 | seat the jaw | descente lente jusqu'à `grasp_z` = 1,8 cm (mâchoire dans le trou) | 1,5 s |
| 7 | close gripper | fermeture | 1 s |
| 8 | hold | pause | 0,5 s |
| 9 | lift | remontée à `travel_z` | 3 s |
| 10 | move above box | translation au-dessus de la boîte | 3,5 s |
| 11 | lower | descente jusqu'à `release_z` = 11 cm | 1,5 s |
| 12 | release | ouverture | 1 s |
| 13 | let it fall | pause | 1 s |
| 14 | up | remontée | 1,5 s |
| 15 | back to rest | retour en position de repos `(0.20, 0, 0.17)` | 3,5 s |

Si aucune prise verticale n'est possible, ou si la boîte est hors d'atteinte (erreur > 1 cm), la planification
échoue avec un message d'erreur, et le bras ne bouge pas.

---

## 7. Driver : exécution

`driver_node` reçoit `/joint_command`, borne les valeurs, puis appelle `send_action()` à 50 Hz. Il lit ensuite
`get_observation()` et publie `/joint_states`. En simulation, chaque appel fait aussi avancer la physique MuJoCo :
les servos simulés suivent la consigne avec un léger retard, et la pince serre réellement le cube (contacts physiques).

---

## 8. Lancer un grab

**Avec le panneau** (http://localhost:8080, lancé par `./scripts/start.sh`), section *Grab d'objet* :
1. *Perception simulée* activée (tant que `perception_node` n'existe pas) ;
2. *Nouveau cube* : position aléatoire ;
3. *Pick & place* : lance la séquence ; l'état du brain s'affiche en direct ;
4. *Repos* : ramène le bras en position de repos.

**En ligne de commande** (dans `docker compose exec control bash`, après les `source`) :

```bash
ros2 param set /so101_driver fake_perception true        # position vérité terrain (temporaire)
ros2 service call /reset_object std_srvs/srv/Trigger     # nouveau cube aléatoire
ros2 service call /pick_and_place std_srvs/srv/Trigger   # lancer le grab
ros2 service call /go_to_rest std_srvs/srv/Trigger       # position de repos
```

> Si `ros2 param set` répond `Node not found`, relancer la commande : la découverte DDS du CLI est parfois lente.

**Mesuré (2026-10-09, sim) :** cube parti de `(0.180, 0.141)` et arrivé en `(0.064, -0.257)`, pour une boîte
centrée en `(0.05, -0.25)` : le cube est bien dans la boîte, à environ 1,6 cm du centre.

---

## 9. Prochaines étapes

1. **`perception_node` OpenCV** (section 4) : HSV → contour → centre → rayon → plan `z = 0.025`, publié sur `/object_position`.
2. Mesurer l'erreur perception vs `get_object_position()` sur plusieurs positions aléatoires.
3. Désactiver `fake_perception` et faire tourner le pick & place **uniquement sur la caméra**.
4. Remplacer `get_camera_extrinsics()` par une **calibration** maison (ArUco + `solvePnP`).
5. Passer au **robot réel** : caméra USB, calibration intrinsèque, mêmes nœuds grâce à `use_sim`.



Afficher une boule au end-effecter du bras-> forward kinematics 
