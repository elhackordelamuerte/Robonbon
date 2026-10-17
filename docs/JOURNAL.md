# Journal de résolution — SO-ARM101

Chaque étape résolue y est documentée : **objectif**, **problème rencontré**, **cause**, **solution**, **vérification**.
Les étapes les plus récentes sont en bas.

Sommaire :

1. [Lancer la stack Docker sur macOS](#étape-1--lancer-la-stack-docker-sur-macos)
2. [Afficher RViz sur macOS](#étape-2--afficher-rviz-sur-macos)
3. [Pouvoir cliquer dans RViz (noVNC)](#étape-3--pouvoir-cliquer-dans-rviz-novnc)
4. [Afficher le bras complet sans driver (`fake_joints`)](#étape-4--afficher-le-bras-complet-sans-driver-fake_joints)
5. [Implémenter `driver_node` et faire bouger le bras (jalon S2)](#étape-5--implémenter-driver_node-et-faire-bouger-le-bras-jalon-s2)
6. [Scripts pour tout lancer / tout arrêter](#étape-6--scripts-pour-tout-lancer--tout-arrêter)
7. [Persistance de la configuration RViz](#étape-7--persistance-de-la-configuration-rviz)
8. [Panneau web de pilotage du bras](#étape-8--panneau-web-de-pilotage-du-bras)
9. [Récupérer la branche d'Arthur (pick & place en sim)](#étape-9--récupérer-la-branche-darthur-pick--place-en-sim)
10. [Doc du grab par caméra + bouton grab dans le panneau](#étape-10--doc-du-grab-par-caméra--bouton-grab-dans-le-panneau)
11. [perception_node OpenCV : grab guidé par la caméra (sim)](#étape-11--perception_node-opencv--grab-guidé-par-la-caméra-sim)
12. [Caméra réelle (webcam USB) dans la perception — mode 2D](#étape-12--caméra-réelle-webcam-usb-dans-la-perception--mode-2d)
13. [Port 8081 occupé + grab lancé sur une position périmée](#étape-13--port-8081-occupé--grab-lancé-sur-une-position-périmée)
14. [Piloter le bras réel depuis le Mac (pont série) + version du créateur](#étape-14--piloter-le-bras-réel-depuis-le-mac-pont-série--version-du-créateur)
15. [Panneau : passage simulation ↔ bras réel](#étape-15--panneau--passage-simulation--bras-réel)
16. [Calibration du bras réel et premier démarrage](#étape-16--calibration-du-bras-réel-et-premier-démarrage)
17. [Robustesse : déconnexion USB, mauvais port, caméra choisie par son nom](#étape-17--robustesse--déconnexion-usb-mauvais-port-caméra-choisie-par-son-nom)
18. [Calibration de la caméra réelle avec l'ArUco seul](#étape-18--calibration-de-la-caméra-réelle-avec-laruco-seul)
19. [Premier grab réel et détection du bol de dépôt](#étape-19--premier-grab-réel-et-détection-du-bol-de-dépôt)
20. [Épaule qui cale sous la gravité + pose caméra multi-marqueurs](#étape-20--épaule-qui-cale-sous-la-gravité--pose-caméra-multi-marqueurs)
21. [Pipeline automatique : zone délimitée par 4 ArUco avant le grab](#étape-21--pipeline-automatique--zone-délimitée-par-4-aruco-avant-le-grab)
22. [Retour à l'état d'avant les 4 ArUco de cartographie](#étape-22--retour-à-létat-davant-les-4-aruco-de-cartographie)
23. [Vérification de la prise par la caméra et relance automatique](#étape-23--vérification-de-la-prise-par-la-caméra-et-relance-automatique)
24. [Jeu Pierre - Feuille - Ciseaux (Chifoumi) avec caméra réelle et bras simulé](#étape-24--jeu-pierre---feuille---ciseaux-chifoumi-avec-caméra-réelle-et-bras-simulé)

---

## Étape 1 — Lancer la stack Docker sur macOS

**Objectif :** suivre la slide « Setup » (build, `up`, logs, shell dans `control`).

**Problèmes :**
- `docker compose up -d control perception viz` : le service `perception` n'existe pas dans
  `docker/docker-compose.yml` (seuls `control` et `viz` sont définis).
- `xhost +local:docker` : `xhost` n'existe pas sur macOS sans XQuartz.

**Solution :**
```bash
cd docker
docker compose build                  # une fois
docker compose up -d control viz
docker compose logs -f control        # attendre "Brain node ready."
```

**Vérification :** logs `Brain node ready.` et `Driver node ready.` (mode simulation, `USE_SIM=true` par défaut).

---

## Étape 2 — Afficher RViz sur macOS

**Objectif :** lancer `rviz2 -d /ros2_ws/so101.rviz` depuis le conteneur `control`.

**Tentative 1 — XQuartz (X11), comme sur Linux :**

| Essai | Erreur | Cause |
|---|---|---|
| `DISPLAY` du compose | `could not connect to display` | `DISPLAY` vide sur Mac, et le socket `/tmp/.X11-unix` n'est pas partagé avec la VM Docker. |
| `DISPLAY=host.docker.internal:0` + `xhost + 127.0.0.1` | `Authorization required` | La connexion venant de Docker n'arrive pas depuis 127.0.0.1. Corrigé en écrivant le cookie MIT-MAGIC-COOKIE dans un `.Xauthority` (famille *Wild*) dans le conteneur. |
| Connexion X11 OK | `OpenGL 1.5 is not supported` / `No matching fbConfigs` | Le GLX de XQuartz (même avec `+iglx`, en indirect comme en logiciel) ne fournit pas un OpenGL utilisable par Ogre (moteur de RViz). **Impasse.** |

**Solution retenue — RViz dans un écran virtuel, affiché dans le navigateur :**
un conteneur annexe `tek5-rviz` (image `tek5/control`) sur le réseau `tek5_rosnet` lance :
- `Xvfb` : écran X virtuel, rendu OpenGL logiciel Mesa (OpenGL 4.5) ;
- `x11vnc` + `websockify`/noVNC : affichage sur http://localhost:6080 ;
- `rviz2` en boucle (redémarre s'il est fermé).

Commande complète : voir [SETUP_MAC.md](SETUP_MAC.md#3-lancer-rviz-dans-le-navigateur).

**Vérification :** log RViz `OpenGl version: 4.5`. Le conteneur voit `/tf` et `/tf_static`
publiés par `viz`, donc la découverte DDS entre conteneurs fonctionne.

---

## Étape 3 — Pouvoir cliquer dans RViz (noVNC)

**Problème :** RViz s'affiche mais ne réagit pas à la souris. noVNC s'est aussi figé une fois.

**Diagnostic :**
- `x11vnc` indique que XTEST est actif, donc les clics arrivent bien au serveur X.
- Sans gestionnaire de fenêtres, la fenêtre Qt n'a pas le focus, et la gestion souris
  XInput2 de Qt ignore les événements injectés par VNC.

**Solution :** ajouter un gestionnaire de fenêtres léger et désactiver XInput2 côté Qt :
- paquet `fluxbox`, lancé après `Xvfb` ;
- variable `-e QT_XCB_NO_XI2=1`.

Si noVNC se fige : recharger la page, sinon `docker restart tek5-rviz`.

**Vérification :** clic simulé (`xdotool`) sur le menu *File* + capture d'écran (`import`) : le menu s'ouvre.

---

## Étape 4 — Afficher le bras complet sans driver (`fake_joints`)

**Problème :** le bras apparaît « en morceaux » dans RViz.

**Cause :** `robot_state_publisher` ne publie les TF des joints mobiles qu'à partir de
`/joint_states`, or `driver_node` ne publiait encore rien (squelette à compléter).

**Solution :** argument `fake_joints` (par défaut `false`) dans
`ros2_ws/src/so101_driver/launch/driver.launch.py`, qui lance `joint_state_publisher` (pose zéro) :
```bash
ros2 launch so101_driver driver.launch.py fake_joints:=true
```

**Vérification :** `/joint_states` à environ 10 Hz ; `/tf` contient les 6 links (`shoulder_link`,
`upper_arm_link`, `lower_arm_link`, `wrist_link`, `gripper_link`, `moving_jaw_so101_v1_link`).

> ⚠️ Ne plus utiliser une fois `driver_node` implémenté (étape 5) : deux sources publieraient `/joint_states`.

---

## Étape 5 — Implémenter `driver_node` et faire bouger le bras (jalon S2)

**Objectif ([SUJET.md](SUJET.md)) :** `driver_node` choisit `SO101Sim` ou `SO101Follower` selon `use_sim`,
publie `/joint_states` (≥ 20 Hz) et `/external_cam/image_raw` (~10 Hz), et s'abonne à `/joint_command`
(radians, pince 0-100). Jalon S2 : le bras bouge sur un `ros2 topic pub /joint_command` manuel.

**Fichier :** `ros2_ws/src/so101_driver/so101_driver/driver_node.py`

**Conception :**

| Élément | Choix |
|---|---|
| Backend | `SO101Sim()` si `use_sim`, sinon `SO101Follower(SO101FollowerConfig(port, id="arm", calibration_dir=/calibration, use_radians=True))`. Erreur explicite si la calibration est absente. `connect(calibrate=False)`. |
| `/joint_command` | `JointState` : seuls les joints présents dans le message sont mis à jour (les autres gardent leur cible). Les joints inconnus et les messages mal formés sont ignorés avec un warning. |
| Boucle de contrôle | Timer à 50 Hz : `send_action(cible)` puis `get_observation()`. En sim, ces appels font aussi avancer MuJoCo. |
| `/joint_states` | Timer à 30 Hz, en radians. Pince convertie de % vers l'angle URDF (`GRIPPER_RANGE_RAD`). |
| Caméra | Timer à 10 Hz, sim uniquement : `render_camera()` → `cv_bridge` (`rgb8`, `frame_id=external_cam`). |
| Arrêt | `destroy_node()` déconnecte le backend (couple désactivé sur le bras réel). |

**Problème rencontré :** `ros2 topic pub --once ...` ne faisait pas bouger le bras, même avec `-w 1`.

**Diagnostic :**
1. `ros2 topic info -v /joint_command` : le driver est bien abonné.
2. Test direct de `SO101Sim` en Python (sans ROS) : la sim atteint la consigne, donc le backend est OK.
3. Publication en continu (`-r 10 --times 20`) : le bras bouge.

**Cause :** avec `--once`, la CLI se ferme avant que la connexion DDS soit établie, et le message unique est perdu.

**Commande de test :**
```bash
ros2 topic pub -r 10 --times 10 /joint_command sensor_msgs/msg/JointState \
  "{name: [shoulder_pan, shoulder_lift, elbow_flex, gripper], position: [0.6, -0.4, 0.5, 90.0]}"
```

**Vérification :**
- `ros2 topic hz /joint_states` → 30,8 Hz ✅ (≥ 20 Hz)
- `ros2 topic hz /external_cam/image_raw` → 10,06 Hz ✅
- `/joint_states` après la commande → `[0.600, -0.400, 0.500, 0.000, 0.000, 1.553]` ✅
  (1.553 rad = pince ouverte à 90 %)
- `ruff check` → aucune erreur
- Mode bras réel **non testé** : pas de bras branché, et pas de passthrough USB avec Docker Desktop sur Mac.

**Prochaine étape (S4) :** IK dans `brain_node`, pour atteindre un point XYZ arbitraire à moins de 1 cm près.

---

## Étape 6 — Scripts pour tout lancer / tout arrêter

**Objectif :** remplacer la suite de commandes manuelles (compose, attente des logs, conteneur RViz) par deux scripts.

**Fichiers :** `scripts/start.sh`, `scripts/stop.sh`

```bash
./scripts/start.sh            # control + viz + RViz (noVNC), ouvre le navigateur
./scripts/start.sh --build    # reconstruit l'image avant
./scripts/start.sh --no-rviz  # sans RViz
./scripts/stop.sh             # arrête tout
```

**`start.sh` :**
1. vérifie que Docker tourne ;
2. `docker compose up -d control viz` ;
3. attend `Driver node ready` dans les logs, et s'arrête avec une erreur si un `Traceback` apparaît (délai max 180 s) ;
4. recrée le conteneur `tek5-rviz` (Xvfb + fluxbox + noVNC, cf. étapes 2-3), attend `PRET_RVIZ`, puis ouvre http://localhost:6080 ;
5. reprend le `ROS_DOMAIN_ID` de `docker/.env` s'il existe (sinon 42), pour que RViz soit sur le même domaine que les autres conteneurs.

**`stop.sh` :** supprime d'abord `tek5-rviz`, puis lance `docker compose down`.
L'ordre compte : `tek5-rviz` est branché sur le réseau `tek5_rosnet`, ce qui empêcherait sinon `compose down` de supprimer ce réseau.
L'arrêt envoie SIGTERM : l'entrypoint relaie le signal et le driver déconnecte proprement le backend.

**Vérification :**
- `stop.sh` → conteneurs et réseau `tek5_rosnet` supprimés ✅
- `start.sh` → `tek5-control-1`, `tek5-viz-1` et `tek5-rviz` actifs ; `rviz2` tourne ; `/joint_states` à 30 Hz vu depuis le conteneur RViz ✅

---

## Étape 7 — Persistance de la configuration RViz

**Problème :** les modifications faites dans RViz (displays ajoutés, vue, panneaux) sont perdues à chaque relance.

**Causes :**
1. RViz n'écrit la config que sur *File > Save Config* (Ctrl+S). Or `stop.sh` supprime le conteneur sans laisser RViz proposer d'enregistrer.
2. Les préférences RViz (`/root/.rviz2` : configs récentes, état des fenêtres) sont stockées dans le conteneur, et supprimées avec lui.

Le fichier de config lui-même (`/ros2_ws/so101.rviz`) est bien monté depuis l'hôte (`ros2_ws/so101.rviz`) et accessible en écriture : le problème n'est pas là.

**Solution :**
- `stop.sh` : avant de supprimer `tek5-rviz`, envoie **Ctrl+S** à la fenêtre RViz avec `xdotool`, ce qui enregistre dans `ros2_ws/so101.rviz` sur l'hôte. Un avertissement s'affiche si l'enregistrement échoue (RViz fermé, par exemple).
- `start.sh` : installe `xdotool` dans le conteneur et monte un volume Docker nommé `tek5-rviz-config` sur `/root/.rviz2`.
- En cours de session : **Ctrl+S** dans RViz enregistre immédiatement.

> `ros2_ws/so101.rviz` est versionné : les changements apparaissent dans `git diff`.
> Pour revenir à la config d'origine : `git checkout ros2_ws/so101.rviz`.

**Vérification :**
- Ctrl+S via `xdotool` → la date de modification de `ros2_ws/so101.rviz` change, et `git diff` montre la vue et les panneaux enregistrés ✅
- `stop.sh` → message « Config RViz enregistrée », date du fichier mise à jour ✅
- `start.sh` → `docker inspect tek5-rviz` montre bien `tek5-rviz-config -> /root/.rviz2` ✅

---

## Étape 8 — Panneau web de pilotage du bras

**Objectif :** animer le bras plus simplement qu'en tapant des `ros2 topic pub /joint_command ...`.

**Fichiers :** `ros2_ws/tools/joint_panel/` (`panel.py`, `index.html`, `poses.json`). Lancé par `start.sh`
dans le conteneur `tek5-panel`, sur **http://localhost:8080**.

**Fonctionnalités :**
- un slider et un champ numérique par joint, bornés aux limites du URDF (pince 0-100 %), avec la position mesurée à côté ;
- *Envoi direct* (le bras suit le slider, envois limités à 10 par seconde) ou bouton *Envoyer* ;
- *← Position actuelle* (recopie l'état mesuré dans les sliders) et *Position zéro* ;
- **poses** nommées, enregistrées dans `poses.json` : *Aller*, *+ Séquence*, supprimer ;
- **séquences** : liste de poses avec une durée par étape, ordre modifiable, lecture simple ou en boucle, *Stop*,
  enregistrement sous un nom ;
- la **commande `ros2 topic pub` équivalente** est affichée en direct (bouton *Copier*), pour faire le lien avec le CLI.

**Conception :**

| Élément | Choix |
|---|---|
| Serveur | Nœud rclpy `joint_panel` + `http.server` de la bibliothèque standard : aucune dépendance à installer. `rclpy.spin` tourne dans un thread, le serveur HTTP dans le thread principal. |
| API | `GET /api/etat`, `GET/POST /api/poses`, `POST /api/commande`, `POST /api/lecture`, `POST /api/stop`. Valeurs bornées côté serveur ; requête vide ou mal formée → erreur 400. |
| Animation | Interpolation linéaire côté serveur, depuis la position **mesurée** vers chaque pose, publiée à 30 Hz sur `/joint_command`. Une nouvelle commande manuelle interrompt la lecture en cours. |
| Règle du sujet | Le panneau passe uniquement par `/joint_command`, sans accès direct au backend. |

**Problèmes rencontrés :**
1. `start.sh` signalait un délai dépassé alors que le panneau était prêt. **Cause :** le logger ROS écrit sur
   stderr, et la fonction d'attente faisait `2>/dev/null`. **Correction :** `2>&1` dans `attendre_log`.
2. Pendant le test, `shoulder_pan` ne tournait presque pas. **Cause :** une consigne précédente
   (`shoulder_lift` à la limite de 1.745 rad) plaquait le bras contre le sol, et le frottement bloquait la rotation.
   Ce n'est pas un bug du panneau : en remettant le bras au repos, tout bouge normalement.
   ⚠️ Toutes les positions dans les limites du URDF ne sont pas atteignables : le bras peut entrer en collision avec la table.

**Vérification :**
- `GET /` → 200 ; `/api/etat` renvoie les 6 joints ✅
- `POST /api/commande` avec `shoulder_lift: 15` → valeur bornée à 1.745, et le bras bouge ✅
- Séquence de 2 poses (1,5 s chacune) → `en_lecture` passe à vrai puis à faux ; pose finale atteinte
  (`pan=-0.90`, `roll=-0.80`, pince 80 %) ✅. Les servos simulés suivent la consigne avec un peu de retard.
- `POST /api/lecture` avec une séquence vide → 400 `séquence vide` ✅
- `ruff check` → aucune erreur

`poses.json` contient des poses d'exemple (`repos`, `lever`, `gauche`, `droite`) et une séquence `démo`.

---

## Étape 9 — Récupérer la branche d'Arthur (pick & place en sim)

**Objectif :** repartir du travail d'Arthur, qui a déjà fait un grab en simulation.

**Constat :** pas de branche nommée « arthur ». Son travail est sur `origin/test-bootstrap` (11 commits d'Arthur Bourdin) :
driver (caméra + `camera_info`, `fake_perception`, `/reset_object`), cinématique (`kinematics.py`), brain (IK de prise
verticale + machine d'états du pick & place), config RViz avec panneau caméra, service `perception` dans le compose.

**Démarche :**
1. Les modifications locales non commitées (driver avec `is_calibrated`, « pose 5 », vue RViz) ont été mises de côté :
   `git stash` → `stash@{0}` (« local avant passage sur test-bootstrap »).
2. `git checkout -b test-bootstrap --track origin/test-bootstrap`.
3. Docker Desktop était arrêté (`docker.sock` introuvable) : relancé avec `open -a Docker`.
4. `ros2 param set /so101_driver fake_perception true` a d'abord répondu `Node not found` : la découverte DDS du CLI
   est lente, il suffit de relancer la commande.

**Constat important :** le grab d'Arthur **n'utilise pas encore la caméra** pour localiser le cube. `/object_position`
vient de `fake_perception`, c'est-à-dire de la vraie position lue dans MuJoCo (`get_object_position()`, **interdit**
dans le pipeline final). `so101_perception` est vide et désactivé (`COLCON_IGNORE`).

**Vérification :** `/pick_and_place` → les 15 étapes s'enchaînent ; cube parti de `(0.180, 0.141)` et arrivé en
`(0.064, -0.257)`, pour une boîte en `(0.05, -0.25)` ✅

---

## Étape 10 — Doc du grab par caméra + bouton grab dans le panneau

**Branche :** `feat/grab-camera` (partie de `test-bootstrap`, avec le panneau, les scripts et la doc ramenés depuis
`feat/panneau-pilotage`). Le driver d'Arthur est conservé.

**Doc :** [GRAB_CAMERA.md](GRAB_CAMERA.md). Elle explique l'architecture des nœuds, les mécanismes ROS 2 (topics,
services, paramètres, timers, TF2, cv_bridge, DDS), le modèle sténopé et la matrice K, la perception OpenCV à écrire
(HSV → contour → moments → rayon → intersection avec le plan de la table, conventions MuJoCo/OpenCV, calibration),
l'IK par moindres carrés pondérés, la planification et les 15 états du pick & place.

**Panneau (http://localhost:8080), nouvelle carte *Grab d'objet* :**

| Contrôle | Mécanisme ROS 2 |
|---|---|
| *Perception simulée* (case à cocher) | Service `/so101_driver/set_parameters` (`SetParameters`, `fake_perception`) ; état relu via `get_parameters` |
| *Nouveau cube* | Service `/reset_object` (`Trigger`) |
| *Pick & place* | Service `/pick_and_place` (`Trigger`) |
| *Repos* | Service `/go_to_rest` (`Trigger`) |
| Positions cube / boîte | Abonnement à `/object_position` et `/drop_box_position` |
| Journal du brain | Abonnement à `/rosout`, filtré sur le nœud `so101_brain` (8 derniers messages) |

Les appels de service se font depuis le thread HTTP : `call_async`, puis on attend la réponse (timeout 5 s → HTTP 503)
pendant que `rclpy.spin` tourne dans un autre thread. Un grab interrompt d'abord une éventuelle animation du panneau.

**Vérification :**
- `POST /api/perception {actif: true}` → succès ; `GET /api/grab` renvoie `fake_perception: true` ✅
- *Nouveau cube* → `object at (0.245, -0.125)` ✅
- *Pick & place* → planification lancée ; un 2e appel pendant l'exécution → `already running` ✅
- Journal du brain affiché jusqu'à `pick & place: done` ; cube final `(0.065, -0.256)`, dans la boîte `(0.05, -0.25)` ✅
- Action inconnue → HTTP 400 ✅ ; `ruff check` → aucune erreur

**Ajout : commande ROS 2 affichée.** Sous les boutons, un bloc *Commande ROS 2 exécutée* montre l'équivalent CLI
de la dernière action, avec le `source` à faire avant et un bouton *Copier* :

| Action du panneau | Commande affichée |
|---|---|
| Perception simulée | `ros2 param set /so101_driver fake_perception true` (ou `false`) |
| Nouveau cube | `ros2 service call /reset_object std_srvs/srv/Trigger` |
| Pick & place | `ros2 service call /pick_and_place std_srvs/srv/Trigger` |
| Repos | `ros2 service call /go_to_rest std_srvs/srv/Trigger` |

Au chargement de la page, le bloc affiche la séquence complète (perception → nouveau cube → pick & place).
Vérifié : chaque commande, lancée à la main dans `control`, fonctionne (`Set parameter successful`,
`Trigger_Response(success=True, ...)`) ; `node --check` du JavaScript → OK.

---

## Étape 11 — perception_node OpenCV : grab guidé par la caméra (sim)

**Objectif :** remplacer `fake_perception` (vérité terrain, interdite) par une vraie perception à partir de l'image.

**Point de départ :** aucune implémentation sur aucune branche, seulement un squelette `perception_node.py`
(qui parlait encore d'une « balle dorée ») et un paquet désactivé (`COLCON_IGNORE`).

**Prototype hors ROS** (scripts lancés dans `control` sur `SO101Sim`, 40 à 60 tirages aléatoires) :

| Constat | Correction |
|---|---|
| Rouge du cube : H ≈ 172, S ≈ 235, V ≈ 180 | Deux plages HSV : 0-12 et 160-180 |
| Erreur de 43 cm sur un tirage : le **marron de la boîte** (H≈10, V≈73) tombait dans la plage basse du rouge | Seuil `V ≥ 100` |
| Erreurs de 5 cm : cube **partiellement caché** par la pince, ou **coupé par le bord** de l'image | Contour qui touche le bord → image ignorée |
| Erreur selon le plan d'intersection : z=1,5 cm → 13 mm ; z=2,5 cm → 4,2 mm ; z=3 cm → 4,3 mm | Plan `z = 2,75 cm` (le centre visible mélange le dessus et les faces) |
| Yaw (contour projeté sur le dessus du cube, `minAreaRect`) : médiane 0,9°, max 22° | Gardé tel quel |
| 12 tirages sur 60 : cube **hors du champ** de la caméra | Limite de la scène (rien à corriger côté code) |

**Implémentation** (`so101_perception/perception_node.py`) :
- abonnements à `/external_cam/image_raw` et `/camera_info` (K) ; pose caméra `cam_T` lue dans les paramètres du driver
  via `GetParameters`, de façon asynchrone, comme le prévoyait le squelette ;
- HSV → fermeture morphologique → plus grand contour → moments → rayon `K⁻¹[u,v,1]` → conversion MuJoCo→monde → plan z ;
- publie `/object_position` (avec le yaw), `/drop_box_position` et `/perception/debug` (image annotée) ;
- conversion d'image **avec numpy, sans `cv_bridge`** : l'image `perception` installe numpy 2 avec opencv, alors que
  `cv_bridge` (apt) est compilé pour numpy 1.

**Autres changements :**
- driver (sim) : paramètre `cam_T` = `get_camera_extrinsics()` (autorisé pour démarrer ; à remplacer par une calibration) ;
- compose : le service `perception` lance `ros2 run so101_perception perception` (après le build fait par `control`) ;
- `COLCON_IGNORE` supprimé ; `package.xml` mis à jour.

**Problème : la boîte, estimée à 4,7 cm de sa vraie position.** L'image annotée montre qu'**en position de repos, la pince
masque la boîte**. Le calcul confirme que le rayon caméra → boîte passe par (0.20, 0.01, 0.22), juste à côté du TCP de
repos d'Arthur (0.20, 0, 0.17).
- Brain : position de repos décalée à `(0.18, 0.10, 0.17)` ;
- Perception : la boîte est fixe, donc on garde l'observation **de plus grande aire**, et le centre du rectangle d'aire
  minimale est projeté à z = 3,6 cm (haut des parois).
- Résultat : (0.057, -0.240) pour (0.05, -0.25), soit environ 1,2 cm d'erreur ✅

**Vérification (6 tirages, `fake_perception = false`) :**

| Essai | Vraie position | Caméra | Résultat |
|---|---|---|---|
| 1 | (0.288, 0.072) | — | cube hors champ, essai sauté |
| 2 | (0.198, 0.007) | (0.199, 0.006) | dans la boîte ✅ |
| 3 | (0.264, 0.118) | — | cube hors champ, essai sauté |
| 4 | (0.183, 0.001) | (0.184, 0.000) | dans la boîte ✅ |
| 5 | (0.263, -0.051) | (0.265, -0.052) | dans la boîte ✅ |
| 6 | (0.163, 0.127) | (0.165, 0.127) | dans la boîte ✅ |

**4 réussites sur 4 essais où le cube est visible.** Erreur caméra : 1 à 2 mm. La position finale a été vérifiée en
réactivant `fake_perception` brièvement après chaque essai.

---

## Étape 12 — Caméra réelle (webcam USB) dans la perception — mode 2D

**Objectif :** utiliser une vraie caméra avec OpenCV dans la perception. Choix validés avec l'utilisateur : **webcam USB
externe** (position fixe, nécessaire pour la calibration), et **d'abord le flux + la détection 2D**.

**Contrainte :** Docker Desktop (macOS) ne donne pas accès aux webcams depuis les conteneurs.

**Solution :**

| Fichier | Rôle |
|---|---|
| `scripts/camera_stream.py` | Sur le Mac (`uv run`, dépendances déclarées en tête du script) : capture AVFoundation → MJPEG sur `127.0.0.1:8081`. `--list` liste les caméras. |
| `so101_perception/camera_node.py` | Dans le conteneur : lit `http://host.docker.internal:8081/stream.mjpg` (ou `/dev/videoN` sous Linux), publie `/camera/image_raw` et `/camera/camera_info` (K issue d'un YAML de calibration, sinon approximée avec `hfov_deg`). Reconnexion automatique. |
| `so101_perception/launch/perception.launch.py` | `camera:=sim` (défaut) ou `camera:=real` (+ `source`, `calibration_file`) |
| `perception_node.py` | Paramètres `camera_ns`, `cam_T`, `cam_T_depuis_driver`, `convention_mujoco`, `hsv_rouge`, `aire_min_cube_px`, `detecter_boite`. **Mode 2D** sans `cam_T` : détection et image annotée, sans position 3D. |
| compose / `start.sh` | Le service `perception` lance le launch file avec `CAMERA`, `CAMERA_SOURCE`, `CAMERA_CALIBRATION` ; `start.sh` démarre aussi `perception` (et construit l'image si besoin). |

**Problèmes rencontrés :**
1. `cv2.VideoCapture(0)` échouait : OpenCV essayait un autre backend (`OBSENSOR`). → backend **`CAP_AVFOUNDATION`** forcé sur macOS.
2. Index des caméras : 0 = UGREEN (USB), 2 = caméra intégrée du MacBook (identifiées sur une capture).
3. Premier flux **noir** en 640×360 : un autre périphérique avait été ouvert (l'ordre AVFoundation change, par exemple avec la
   caméra Continuité de l'iPhone). Après relance : 640×480, image correcte. → vérifier l'aperçu sur http://localhost:8081.
4. `ros2 launch` refuse `calibration_file:=` (valeur vide) → valeur par défaut `none`.
5. Piège évité : en mode réel, la perception aurait utilisé le `cam_T` **de la caméra simulée** (lu dans le driver).
   → `cam_T_depuis_driver: false` en mode réel.
6. Fausse détection d'une petite tache rougeâtre en 640×480 → `aire_min_cube_px` réglable (400 en réel).
7. Lecture de `docker/.env.example` refusée par les permissions : fichier non modifié, valeurs par défaut placées dans le compose.

**Vérification :**
- Flux reçu sur le Mac (257 ko en 3 s) **et depuis un conteneur** via `host.docker.internal` (640×480) ✅
- `CAMERA=real` : `/camera/image_raw` et `/perception/debug` à 10 Hz ; `Intrinsics K received` ; pas de `cam_T` → mode 2D ✅
- Détection : une tache au bord de l'image est correctement **ignorée** (contour coupé). Aucun cube rouge dans le champ
  pendant le test (webcam tournée vers le plafond) : **détection réelle d'un cube rouge pas encore validée**.
- `start.sh` (sim) : perception reçoit K et `cam_T`, et publie `/object_position` ✅ ; bascule `CAMERA=real` OK ✅
- `ruff check` → aucune erreur

**Doc :** [PERCEPTION_ROS.md](PERCEPTION_ROS.md) suit le parcours d'une image jusqu'au mouvement du bras, avec la référence des
topics, des services et des paramètres, et les commandes d'observation.

**Reste à faire :** calibration au damier (K), pose caméra par ArUco + `solvePnP` (`cam_T`), réglage HSV sous l'éclairage réel.

---

## Étape 13 — Port 8081 occupé + grab lancé sur une position périmée

**Problème 1 :** `uv run camera_stream.py --index 0` → `OSError: [Errno 48] Address already in use`.
**Cause :** une diffusion lancée pendant les tests de l'étape 12 tournait encore en arrière-plan (`lsof -iTCP:8081`).
**Correction :** processus arrêté (`pkill -f camera_stream.py`). Le script réserve maintenant le port **avant** d'ouvrir la
caméra, et affiche un message clair : `❌ Port 8081 déjà utilisé : un camera_stream tourne sans doute déjà…`.

**Problème 2 : « la caméra réelle dans la simu ne fonctionne pas ».**

*Comportement constaté :* en `CAMERA=real`, *Pick & place* était **accepté** (`object at (0.170, -0.206), planning`) et le bras
partait vers… une ancienne position de vérité terrain, publiée quand `fake_perception` était actif.

*Cause :* en mode réel sans `cam_T`, la perception ne publie pas de `/object_position` (mode 2D). Le brain et le panneau
gardaient **indéfiniment** la dernière valeur reçue.

*Corrections :*
- brain : un pick est refusé si `/object_position` a plus de **2 s** (`OBJECT_MAX_AGE`, âge mesuré à la réception, car les
  horodatages peuvent venir d'une autre horloge). Le principe du sujet est respecté : le contrôle utilise la dernière valeur
  connue, mais un grab ne **démarre** pas sur un objet qui n'est plus vu ;
- panneau : les positions affichent `⚠️ périmée (N s) : plus vu par la perception` au-delà de 2 s.

*Vérification :* position vieille de 4,2 s → `/pick_and_place` répond `success=False`, `"/object_position is 4 s old: object
not seen anymore (perception stopped, cube hidden or out of view?)"` ✅ ; JavaScript `node --check` OK ; `ruff` OK sur les
fichiers modifiés (le seul avertissement restant, B905 `zip()`, est sur une ligne d'origine du brain).

**Limite de fond (pas un bug) :** une caméra réelle filme **le monde réel**, alors que le bras simulé et son cube vivent dans
MuJoCo. Le cube que voit la webcam n'est pas celui de la sim. Pour que l'image réelle pilote le bras simulé
(« jumeau numérique » : le bras simulé va là où est le vrai cube), il faut connaître la **pose de la webcam dans le repère
du robot** (`cam_T`), donc une calibration (marqueur ArUco + `solvePnP`). La sim ne pourra pas pour autant saisir un cube
qui n'existe que dans la réalité.

---

## Étape 14 — Piloter le bras réel depuis le Mac (pont série) + version du créateur

**Contexte :** le jumeau numérique a été mis en pause (cibles déjà générées : `uv run scripts/generate_targets.py` →
`calib_targets/damier_9x6_25mm.pdf` et `aruco_4x4_id0_80mm.pdf`). Priorité demandée : commander **le vrai bras**.

**Contrainte :** pas d'USB dans les conteneurs avec Docker Desktop (macOS) ; le SDK Feetech ouvre le port avec
`serial.Serial(chemin)`, donc il lui faut un vrai fichier de port, pas une URL réseau.

**Solution** (guide : [BRAS_REEL.md](BRAS_REEL.md)) :

| Élément | Rôle |
|---|---|
| `scripts/serial_bridge.py` (Mac, `uv run`, pyserial) | port USB du bras ↔ TCP brut `127.0.0.1:8765`, `TCP_NODELAY`, un seul client, détection auto du port, `--port loop://` pour tester |
| `Dockerfile.control` | nouvelle couche `socat` (seule cette couche est reconstruite) |
| `control_entrypoint.sh` | si `USE_SIM=false` et `SERIAL_BRIDGE` : `socat pty,link=/dev/ttyBRIDGE ↔ tcp:…` (en boucle, avec reconnexion) |
| driver | `max_relative_target` (paramètres `max_step_rad` = 0,05, `max_step_gripper_pct` = 2) → 2,5 rad/s max |
| `brain.launch.py` + entrypoint | argument `rest_on_start`, à `false` sur le bras réel (pas de mouvement au démarrage) |
| `start.sh` | `USE_SIM=false ./scripts/start.sh` : vérifie la calibration et le pont avant de lancer |
| Calibration | **outil officiel LeRobot** : `uvx --from 'lerobot[feetech]' lerobot-calibrate --robot.type=so101_follower --robot.port=… --robot.id=arm --robot.calibration_dir=./calib` → `calib/arm.json` |

**Version du créateur du dépôt (`driver_node.py`)** : comparée à notre driver, la seule différence fonctionnelle est le bloc
`robot.bus.disable_torque()` après la connexion (bras libre, non pilotable ; « *Comment out this line to control the robot* »).
→ bloc **intégré mais commenté**, avec l'explication ; `destroy_node` aligné (docstring, message `Robot disconnected.`).
Le reste (imports en tête, `_connect_arm`, vérification `is_calibrated`) était déjà identique.

**Problèmes rencontrés :**
1. **Port 7000 occupé par macOS** (`ControlCenter`, récepteur AirPlay). Le garde-fou `nc -z` croyait le pont lancé :
   `start.sh` a démarré la stack **en mode bras réel avec une calibration factice**, et le driver a parlé à AirPlay
   (`FeetechMotorsBus motor check failed on port '/dev/ttyBRIDGE'`). → port **8765** ; stack remise en simulation.
2. Ouverture d'un port série invalide : `termios.error` non intercepté → tous les échecs d'ouverture affichent un message clair.
3. `lerobot_min` ne fournit pas la CLI `lerobot-calibrate` → paquet officiel via `uvx` (isolé, lourd car il embarque PyTorch).
   Le script maison `calibrate_arm.py` (basé sur l'« API example » de la doc) a été supprimé au profit de la CLI officielle.
4. DeepWiki et huggingface.co ont répondu HTTP 429 → procédure vérifiée dans la doc source (`docs/source/so101.mdx` sur GitHub).

**Vérification (sans bras branché : aucun `/dev/cu.usbmodem*` présent) :**
- pont `loop://` + socat dans un conteneur : trame Feetech `ff ff 01 04 02 38 02 be` renvoyée à l'identique,
  **0,5 ms** médiane, 11 ms max sur 50 échanges ✅
- `start.sh` en mode réel sans calibration → refus ✅ ; avec calibration mais sans pont → `❌ Pont série injoignable` ✅
- `uvx --from 'lerobot[feetech]' lerobot-calibrate --help` → `--robot.type so101_follower`, `--robot.calibration_dir` disponibles ✅
- image `control` reconstruite avec socat ✅ ; stack sim relancée (`Mode: simulation`, `Driver node ready`) ✅
- **Non testé :** mouvement du vrai bras (pas de bras branché pendant la session).

---

## Étape 15 — Panneau : passage simulation ↔ bras réel

**Objectif :** réunir dans le panneau toutes les informations nécessaires pour passer du bras simulé au bras réel.

**Nouvelle carte *Simulation ↔ Bras réel*** (en tête du panneau) :

| Zone | Contenu |
|---|---|
| Badge | `🖥️ Simulation` / `🦾 BRAS RÉEL` (rouge) / `driver injoignable`, lu dans le paramètre `use_sim` du driver |
| État (✅/❌, bouton *↻ Vérifier*) | driver (mode + port) ; `calib/arm.json` (nombre de moteurs) ; pont série `:8765` joignable ; vitesse max calculée à partir de `max_step_rad` |
| Étapes numérotées | brancher → `lerobot-find-port` → `lerobot-calibrate` → `serial_bridge.py` → `USE_SIM=false ./scripts/start.sh` → vérifier ; commandes **remplies avec le port saisi**, bouton *Copier* |
| Sécurité | espace dégagé, *Envoyer* plutôt qu'*Envoi direct*, vitesse bridée, **le bras retombe à l'arrêt**, contrôles réservés à la sim |
| Retour en simulation | `./scripts/stop.sh && ./scripts/start.sh`, `pkill -f serial_bridge.py` |
| Grab d'objet | *Perception simulée* et *Nouveau cube* **désactivés** en mode bras réel (services propres à la sim) |

Le panneau **vérifie seulement** : changer de mode impose de relancer les conteneurs depuis le Mac.

**Côté serveur :** `GET /api/mode` → paramètres du driver (`use_sim`, `port`, `max_step_rad`, `max_step_gripper_pct`) via
`GetParameters`, présence et contenu de `/calib/arm.json`, test de connexion TCP vers `host.docker.internal:8765`.
`start.sh` crée `calib/` et le monte en lecture seule dans `tek5-panel`.

**Vérification :**
- sim, sans prérequis → `use_sim: true`, `calibration: null`, `pont: false` ✅
- pont `loop://` + `calib/arm.json` factice (6 moteurs) → `calibration: {moteurs: [6]}`, `pont: true` ✅ ; retiré ensuite → retour à `null` / `false` ✅
- `node --check` du JavaScript ✅ ; `ruff` ✅

---

## Étape 16 — Calibration du bras réel et premier démarrage

**Port du bras :** `/dev/tty.usbmodem5A460847701`, VID:PID `1a86:55d3` = puce WCH CH343 de la carte Waveshare (seul port
USB-série). `lerobot-find-port` a échoué une fois (« No difference was found ») : Entrée pressé sans avoir débranché le câble USB.
Inutile : le port était déjà identifié.

**Servos muets** (`scan_port` → `{}`, `lerobot-calibrate` → « Missing motor IDs 1-6 ») : problème **matériel** (alimentation
des servos / câble 3 fils, cf. la doc LeRobot : *the power supply cable might disconnect as you manipulate the board*).
Une fois rebranché : `scan_port` → `{1000000: [1, 2, 3, 4, 5, 6]}`. `lerobot-setup-motors` n'est donc pas nécessaire (id déjà configurés).

**Calibration officielle** (`uvx --from 'lerobot[feetech]' lerobot-calibrate --robot.type=so101_follower … --robot.id=arm
--robot.calibration_dir=./calib`), avec un contrôle automatique de `calib/arm.json` après chaque essai :

| Essai | Défaut constaté | Cause |
|---|---|---|
| 1 | `gripper` 2044→2047 (course nulle) ; `shoulder_lift`, `elbow_flex` 0→4095 | pince non bougée ; « milieu » pris bras replié |
| 2-5 | `shoulder_lift`/`elbow_flex` toujours 0→4095, décalage du coude **-965 à chaque fois** | le décalage enregistre la pose au 1er Entrée : identique 4 fois = bras **replié contre sa butée**, pas en « L » |
| 6 | ✅ toutes les articulations centrées sur ≈ 2048, sans rebouclage | milieu pris bras en « L » |

Diagnostic confirmé par une lecture brute « bras tenu en L » : épaule 292, coude 177, soit ≈ 160° du « milieu » enregistré.
Deux lancements automatiques (« Entrée » envoyés à 30/90 s) n'ont pas aidé : le moment de l'appui compte, à faire à la main.

**Calibration retenue** (`calib/arm.json`, 14:10) : shoulder_pan 765→3362, shoulder_lift 861→3270, elbow_flex 826→3059,
wrist_flex 904→3196, wrist_roll 0→4095, gripper 2030→3571. Vérifiée avec `lerobot_min` (le code du driver) :
`is_calibrated: True` (fichier identique à la mémoire des moteurs), angles au repos cohérents (épaule -96°, coude +98°).

**Premier démarrage du bras réel :** `serial_bridge.py --port /dev/tty.usbmodem5A460847701` puis `USE_SIM=false ./scripts/start.sh`
→ `Mode: hardware (port=/dev/ttyBRIDGE)`, `Robot connected: SOFollower`, `/joint_states` à **30 Hz** depuis le vrai bras.
Le couple s'active, et le bras tient sa pose (pas de mouvement automatique).

**Corrections après démarrage :**
- panneau : « pont non lancé » à tort (le pont n'accepte qu'un client, occupé par le driver) → en mode réel, l'état du pont est
  déduit du driver (`"pont": "driver"`), sans sonde TCP ;
- avertissements en boucle `Failed to get parameters: fake_perception / cam_T` (paramètres propres à la sim, demandés au driver
  réel 1 à 2 fois par seconde) → le panneau ne lit plus `fake_perception` en mode réel ; la perception attend 30 s après un
  refus de `cam_T`. Résultat : 1 avertissement en 20 s au lieu d'environ 30 ;
- `serial_bridge.py` : `print(..., flush=True)` (sortie invisible quand elle est redirigée ; effectif au prochain lancement du pont).

---

## Étape 17 — Robustesse : déconnexion USB, mauvais port, caméra choisie par son nom

**1. Pont série planté sur une micro-déconnexion USB.** `termios.error: (6, 'Device not configured')` dans `reset_input_buffer`
au moment où socat se reconnectait : la carte du bras a disparu un instant (câble ou chute de tension). Le pont s'arrêtait ;
le driver restait ensuite en erreur `Port is in use` (il garde l'ancien pseudo-terminal, et le SDK Feetech reste bloqué).
→ `serial_bridge.py` : erreurs série interceptées (`OSError`, `SerialException`, `termios.error`), **réouverture automatique**
du port (nouvelle tentative chaque seconde), le serveur reste en vie. Le driver, lui, doit être redémarré (`control`) après une
telle coupure : au redémarrage le couple est coupé, **soutenir le bras**.

**2. Pont lancé sur un port inexistant** (`/dev/tty.usbmodem58760431551`) : c'était le port d'exemple de la doc LeRobot, que
j'avais mis par défaut dans le champ « Port du bras » du panneau, et il a été copié tel quel.
→ champ vide par défaut (exemple grisé), commandes affichant `<PORT DU BRAS : étape 2>` tant qu'il est vide, saisie mémorisée
dans le navigateur (`localStorage`, avec repli si indisponible).

**3. Caméra « qui ne fonctionne pas ».**
- la diffusion avait été arrêtée par mon `pkill -f camera_stream.py` lors de la relance du pont ;
- les **index changent** : 3 caméras au lieu de 2 (iPhone en caméra Continuité), et OpenCV et ffmpeg ne les numérotent pas
  dans le même ordre. Un test « objectif caché » a assombri deux index : ambigu.
→ `camera_stream.py --nom UGREEN` : capture **par nom** via ffmpeg (AVFoundation), découpage du flux MJPEG en images JPEG.
  `--list` affiche les noms. Piège corrigé : sans `-video_size`, ffmpeg attendait indéfiniment un mode non servi → capture
  en `1280x720`, puis réduction.

**Vérification :** `--nom UGREEN` → flux 640×480, image correcte de la UGREEN ; cadence mesurée **~5 images/s** (suffisant
pour la perception à 10 Hz ; à améliorer si besoin).

---

## Étape 18 — Calibration de la caméra réelle avec l'ArUco seul

**Objectif :** obtenir la position 3D du cube rouge vu par la UGREEN dans le repère du bras, avec **uniquement le marqueur
ArUco** (pas de damier), comme demandé.

**« Le marqueur n'est pas détecté » :** il l'était (DICT_4X4_50, id 0, **30 images sur 30**, ≈ 67 px de côté). Ce qui bloquait :
`calibrate_pose.py` s'arrête avant toute détection si `ros2_ws/calib/camera.yaml` (intrinsèques) n'existe pas.

**Méthode ArUco seul :**
1. `scripts/calibrate_intrinsics.py` (mode `aruco` par défaut) : ~25 vues du marqueur sous des angles variés → K + distorsion.
   4 coins par vue seulement : modèle **contraint** (`CALIB_FIX_PRINCIPAL_POINT`, `FIX_ASPECT_RATIO`, `ZERO_TANGENT_DIST`,
   `FIX_K3`, estimation initiale f ≈ largeur d'image). Le mode `--cible damier` reste disponible.
2. `scripts/calibrate_pose.py --x … --y …` : marqueur à plat à une position mesurée, flèche +x vers l'avant du robot →
   `solvePnP` (IPPE_SQUARE, médiane sur 30 mesures) → `cam_T` caméra → robot, dans `ros2_ws/calib/camera_pose.yaml`.

**Validation synthétique** (caméra vraie fx = 600, k1 = -0,08, bruit 0,3 px, 25 vues) :
- focale estimée 616,9 (**2,8 %** d'erreur) ; k1 mal estimé (attendu avec 4 points/vue) ;
- **de bout en bout** (K estimée utilisée à la fois pour la pose ArUco et pour projeter le cube) : erreur de position du cube
  **0,1 à 1,3 mm** à 10-18 cm du marqueur. Les erreurs de focale se compensent, car le marqueur et le cube sont sur le même plan.

**Branchement dans la perception :**
- `perception_node` : paramètre `cam_T_file` (YAML de `calibrate_pose.py`), prioritaire sur `cam_T` ;
- `camera_node` : `calibration_file` = `/ros2_ws/calib/camera.yaml` par défaut (repli sur K approximée s'il n'existe pas) ;
- launch / compose : `CAMERA_CALIBRATION` et `CAMERA_POSE`, avec ces chemins par défaut ;
- mode réel : `/drop_box_position` publié depuis une **position fixe** (`boite_fixe`, défaut `[0.05, -0.25]`), car la boîte
  marron de la sim n'existe pas en vrai ; sans elle, le brain refuserait le pick.

**Vérification (UGREEN réelle, pose provisoire puis supprimée) :** le fichier est chargé (`Camera pose from
/ros2_ws/calib/camera_pose.yaml`), et **49 messages en 6 s** sur `/object_position` comme sur `/drop_box_position`.
Note : `ros2 topic echo --once` a affiché « not published yet » alors que les messages arrivaient (découverte lente du CLI).

---

## Étape 19 — Premier grab réel et détection du bol de dépôt

**Flux 640×480 déformé (corrigé) :** `camera_stream.py --nom` réduisait le 1280×720 (16:9) de la UGREEN en 640×480 (4:3) :
pixels non carrés, alors que la calibration ArUco impose fx = fy (deux essais : fx = 662 puis 329, incohérents).
→ `scale=640:-2` : flux **640×360**, format conservé. Les anciennes calibrations sont écartées (`camera_640x480_deforme.yaml.bak`).

**Test « sans nouvelle calibration » (demandé) :** `camera.yaml` **approximatif** (champ supposé 75°, fx = 417, 640×360),
utilisé à la fois pour la pose ArUco et pour la projection du cube.
- `calibrate_pose.py --x 0.25 --y 0` → caméra en x = -0,19 (derrière le bras), **incohérent** avec l'image. L'image de contrôle
  (axes dessinés) montre que la direction base → marqueur suit l'axe **vert** (+y) du marqueur : la flèche « +x » pointait
  vers la **droite** du bras. → `--yaw-deg -90` : caméra en **(0,43 ; 0,44 ; 0,23) m**, devant et à gauche du bras, cohérent.
  Stabilité 0,0-0,9 mm sur 30 mesures.
- Cube localisé de façon stable en (0,165 ; 0,087) m, position confirmée par l'utilisateur.

**Premier pick & place réel :** les 15 étapes se sont enchaînées en 28 s, sans erreur de communication ni contact avec la
table, mais **le cube n'a pas été saisi** (déplacé de 6 mm : la pince l'a frôlé). Causes probables : précision de position
d'environ 1 cm (K approximée) et paramètres de prise réglés en sim (mâchoire dans le trou, `gripper_open` 16 %, `tip_x`,
`grasp_z` 1,8 cm). Proposé : mesurer l'erreur en plaçant la pince 5 cm au-dessus du cube estimé (`/go_to_target`).

**Bol de dépôt (bois) détecté par OpenCV :**
- échantillon HSV du bol : H 13-22, S 44-132, V 88-149 → plage `(10,40,70)-(24,170,200)`, ouverture puis fermeture morphologique ;
- la couleur du bois se retrouve au fond de la salle (meubles, peau) → **filtre de portée** : centre de l'ellipse
  (`fitEllipse`) projeté à mi-hauteur du bol (z = 3 cm), gardé seulement entre 8 et 45 cm de la base. Rejets observés :
  0,7 m, 5 m, 45 m ;
- position lissée (filtre exponentiel 0,3), publiée sur **`/drop_box_position`** (repli sur `boite_fixe` si le bol n'est pas vu) ;
- paramètres `detecter_bol` (vrai en caméra réelle), `hsv_bol`, `z_plan_bol`.

**Vérification :** bol publié en ≈ (0,12 ; -0,20) m (variation de 1-2 cm), soit 12 cm devant et 20 cm à droite de la base ;
cube posé sur le marqueur vu en (0,273 ; 0,055), cohérent avec le centre du marqueur en (0,25 ; 0).

---

## Étape 20 — Épaule qui cale sous la gravité + pose caméra multi-marqueurs

**Test « 5 cm au-dessus du cube »** (script de survol : `ik_grasp` d'Arthur, mouvement interpolé sur 4 s, publication
sur `/joint_command` uniquement) → la pince restait à 2,2 cm du plateau au lieu de 10 cm.

**Diagnostic** (consigne envoyée et mesure 3 s plus tard, articulation par articulation) : `shoulder_lift` restait
**27° sous sa consigne**, les autres à moins de 0,5°. Cause : `max_relative_target` de LeRobot limite l'écart
consigne − **position mesurée** (0,05 rad). Avec le gain P faible des servos, la force est proportionnelle à cet écart :
l'épaule, qui porte tout le bras, **cale sous la gravité**. (Un premier calcul comparatif était faux : l'IK refaite depuis
la pose courante tombait sur une autre solution, poignet retourné de 139°.)

**Correction (driver) :** limiteur de vitesse sur la **consigne envoyée** (`_rate_limited_goal`, au plus `max_step_rad` par
pas de 20 ms, soit 2,5 rad/s), indépendant de la position mesurée. `max_relative_target` est retiré. Bras réel uniquement :
la sim garde le suivi direct.
→ `shoulder_lift` à **3,4°** de sa consigne (au lieu de 27°), TCP à 8,4 cm (visé 10). Reste un léger affaissement statique
(gain P faible), à réduire plus tard.

**« Le bras et la caméra ont du mal avec la profondeur » → 4 marqueurs en rectangle.** Image : les 4 portaient **le même
id 0** (même PDF), donc impossibles à distinguer ; un 4ᵉ était coupé par le bord, et les deux du fond ne faisaient que
16-17 px. La caméra avait aussi été déplacée (pose à refaire).
- `generate_targets.py` : 4 marqueurs **différents** `aruco_4x4_id{0..3}_100mm.pdf` (100 mm, coins plus précis de loin) ;
- `calibrate_pose.py --marqueurs calib/marqueurs.yaml` : positions des centres (repère robot) + orientation commune des
  flèches ; tous les coins des marqueurs vus (≥ 2) sont résolus **ensemble** par `solvePnP` directement dans le repère
  robot. Médiane sur 30 mesures, erreur de reprojection et image de contrôle avec le repère robot dessiné ;
- modèle `calib/marqueurs.yaml` (rectangle 20 × 30 cm à mesurer).

**Validation synthétique** (200 tirages, bruit 0,5 px) : erreur sur la position de la caméra **1,2 mm** en médiane
(95ᵉ centile 2,5 mm) avec 4 marqueurs, contre **8,7 mm** (95ᵉ centile 16 mm) avec un seul, soit 7 fois mieux.

---

## Étape 21 — Pipeline automatique : zone délimitée par 4 ArUco avant le grab

**Demande :** les 4 marqueurs sont **identiques** (même id) et servent juste de repères ; pipeline OpenCV automatique au
clic, avant le grab : centres des 4 ArUco, diagonales, forme de la surface, distances approximatives, sans mesure au mètre.

**Étape intermédiaire (`calibrate_pose.py`)** : les marqueurs identiques produisaient un résultat absurde (caméra à 2,2 m,
reprojection 104 px), car chaque copie de l'id 0 était associée à la même position. → doublons détectés, puis mode
`positions:` sans identifiants : **association automatique** en testant les 24 permutations détection → position
(reprojection minimale ; l'orientation du motif lève l'ambiguïté de symétrie). Synthétique : 100/100 associations
correctes, caméra à 1,5 mm près. Ce mode exige toutefois de mesurer la position des 4 marqueurs.

**Pipeline automatique** (`so101_perception/zone.py`, fonctions pures) :
1. détection des 4 plus grands marqueurs (ids ignorés), centre de chacun ;
2. tri en quadrilatère (angle autour du barycentre), **diagonales** (0-2, 1-3), **centre de zone** = intersection dans
   l'image, et point le plus proche des deux diagonales en 3D ;
3. **échelle** = taille connue d'un marqueur : `solvePnP` (IPPE_SQUARE) par marqueur → centres 3D (repère caméra) →
   plan de la table (SVD), **côtés, diagonales, hauteur de la caméra**, planéité ;
4. **repère de zone** : origine = centre, x = sens moyen des flèches des marqueurs (stable, indépendant de l'ordre des
   points dans l'image), z = normale à la table vers la caméra.

**Intégration :**
- `perception_node` : service **`/perception/analyser_zone`** (Trigger, réponse JSON), zone dessinée 60 s sur
  `/perception/debug` (quadrilatère bleu, diagonales jaunes, centre rouge, longueurs) ; si `centre_zone_robot` est
  renseigné, **`cam_T` est recalculé** (robot ← zone ← caméra) à chaque analyse ;
- `ros2_ws/calib/zone.yaml` (relu à chaque analyse) : `cote_mm`, `centre_zone_robot: [x, y, yaw_deg]`. C'est la seule chose
  que la caméra ne peut pas voir : la position de la zone par rapport au bras ;
- panneau : bouton **🧭 Analyser la zone** ; en mode bras réel, **Pick & place enchaîne automatiquement** analyse de zone,
  attente de 0,6 s (`/object_position` recalculée avec la nouvelle pose), puis grab. Le grab est annulé si les 4 marqueurs
  ne sont pas vus. Commandes CLI affichées.

**Vérification (UGREEN réelle, 4 marqueurs identiques) :** côtés 27,9 / 42,1 / 27,3 / 41,3 cm, diagonales 48,6 / 51,4 cm
(rectangle ≈ 28 × 41 cm, côtés opposés cohérents à 1 cm près), caméra à **25,5 cm** au-dessus de la table, à 70 cm du
centre, planéité **0,6 mm**. Résultats stables entre deux appels. Recompilation de `so101_perception` seule, dans `control`,
sans redémarrer le driver (le bras reste sous couple).

**Réglage de `centre_zone_robot` et robustesse :**
- mesure de l'utilisateur : centre de zone à ≈ 38 cm droit devant le bras, flèches « vers l'avant ». Avec yaw = 0°, la
  caméra se retrouvait à 1,04 m devant le bras, le regardant de face : contredit par l'image (bras vu de côté). → pour
  chaque yaw candidat (0, 90, -90, 180°), **projection de la base du bras dans l'image** : seul **+90°** la place sur le bras
  réel (pixel 447, 216). `centre_zone_robot: [0.38, 0.0, 90.0]` → caméra en (0,38 ; 0,66 ; 0,26) m, à gauche du bras ;
- le bol posé sur un marqueur le masquait (3 marqueurs visibles) → `completer_rectangle` : avec 3 marqueurs, le coin manquant
  est déduit (sommet à angle droit + parallélogramme), tracé « (coin déduit) » ;
- cube annoncé à (0,93 ; -1,58) m : un objet rouge du fond de la salle était pris pour le cube → **filtre de portée** sur le
  cube aussi (plus grand contour rouge entre 5 et 45 cm de la base).

**Résultat :** cube en (0,175 ; 0,097) m, bol en (0,243 ; -0,037) m, stables, cohérents avec l'image.

---

## Étape 22 — Retour à l'état d'avant les 4 ArUco de cartographie

**Demande :** revenir aux modifications d'avant l'implémentation des 4 ArUco de cartographie. Rien n'étant commité, le retour
a été fait à la main, fichier par fichier.

**Retiré :**
- `scripts/calibrate_pose.py` : mode multi-marqueurs (`--marqueurs`, association de marqueurs identiques, permutations,
  détection des doublons) → version à **un seul marqueur** (`--x --y --yaw-deg --marqueur-mm`) ;
- `scripts/generate_targets.py` : marqueurs `aruco_4x4_id{0..3}_100mm` (et leurs PDF/PNG) ; `calib/marqueurs.yaml` ;
- `so101_perception/zone.py`, service `/perception/analyser_zone`, dessin de la zone, `ros2_ws/calib/zone.yaml` ;
- filtre de portée du cube (`_contour_cube`, `PORTEE_CUBE_M`) et coin déduit avec 3 marqueurs (ajoutés pendant cette phase) ;
- panneau : bouton « Analyser la zone », enchaînement automatique zone → grab en mode réel.

**Conservé (antérieur aux 4 ArUco) :** limiteur de vitesse du driver (correction de l'épaule qui calait, étape 20),
détection du bol, calibration ArUco à un seul marqueur, flux caméra 640×360, `camera.yaml` approximatif (champ 75°).

**Vérification :** `ruff` OK ; perception recompilée seule (sans redémarrer le driver) et relancée : `Perception node ready
on /camera`, plus de service `analyser_zone` ; panneau HTTP 200 ; `node --check` du JavaScript OK.

**À refaire :** la caméra ayant été déplacée, `ros2_ws/calib/camera_pose.yaml` n'existe plus (la perception est en 2D) :
relancer `uv run scripts/calibrate_pose.py --x … --y … --yaw-deg … --marqueur-mm 80` avec un marqueur posé et mesuré.

---

## Étape 23 — Vérification de la prise par la caméra et relance automatique

**Contexte (grabs réels après l'étape 22) :** recalibration de pose avec un seul marqueur (orientation -90° confirmée en
projetant la base du bras), filtre de portée du cube remis, caméra déplacée une nouvelle fois puis recalibrée.
`calibrate_pose.py` conserve désormais `correction_objet`. Prise latérale réglée à chaud sur le brain (`gripper_open` 70 %,
`tip_x`/`tip_y` 0, `grasp_z` 2,5 cm, `seat_z` 4,5 cm, via `SetParameters`, la CLI étant trop lente) et correction
`correction_objet: [0, -0.025]` (pince 2-3 cm à gauche du cube d'après l'utilisateur). Les grabs s'exécutent en entier
mais **sans saisir le cube**. Les photos automatiques des étapes (descente, assise, fermeture, levée) montrent la pince
restée au-dessus et en arrière du cube.

**Fonctionnalité demandée :** après chaque grab, le bras levé, la perception vérifie si le cube est dans la pince ; sinon,
nouvel essai avec une rectification de 0,5 (interprétée comme un pas de **0,5 cm**, réglable).

**Perception — service `/perception/verifier_prise`** (Trigger, réponse JSON) :
- position **mesurée** de la pince via `/end_effector_pose` (FK du brain sur `/joint_states` : affaissement compris) ;
- projection dans l'image du segment [TCP, TCP − 4 cm en z] (le cube pend sous la pointe des mâchoires) ;
- plus petite distance entre ce segment et le centre d'un contour rouge (seuil d'aire divisé par 2 : cube en partie masqué
  par les mâchoires) ; **réussie** si elle est inférieure à 35 px, ratée sinon ; « indéterminé » sans image ou sans pose.

**Brain — étape `verify grasp` après `lift`** (paramètre `verify_grasp`, vrai par défaut) :
- bras immobile 0,8 s (`T_CHECK`), puis appel asynchrone au service : le timer à 50 Hz n'est jamais bloqué ;
- **réussie** : la séquence continue (transport, dépôt) et le décalage utilisé est **retenu** pour les grabs suivants ;
- **ratée** : réouverture de la pince, puis **nouvelle planification** sur la position fraîche du cube + décalage de
  recherche suivant : (0, 0), puis des anneaux de `rectification_step` (0,005 m) : (0, −r), (0, +r), (+r, 0), (−r, 0)… ;
- arrêt après `max_grasp_attempts` (6) essais, ou si le cube n'est plus vu (> 2 s) → retour au repos ;
- service indisponible ou verdict indéterminé : la séquence continue, avec un avertissement (pas de boucle infinie).

**Déploiement sans couper le bras :** `colcon build` des deux paquets dans `control`, puis relance du **seul processus du
brain** (`docker exec -d … ros2 launch so101_brain brain.launch.py rest_on_start:=false`) ; le driver garde le couple.
Piège : un premier `pkill -f "so101_brain brain.launch.py"` avait aussi tué le shell qui contenait cette chaîne.

**Vérification :** bras au repos, cube sur la table → `{"verdict": "ratée", "pince_px": [387, 74], "cube_px": [382, 199],
"distance_px": 86.1}` ✅ (bon verdict négatif).

**Premier pick & place réel réussi (cycle complet de relance)** — cube vu en (0,168 ; 0,012) m, bol en (0,183 ; -0,178) m :

| Essai | Décalage de la cible | Pince (mesurée) | Vérification |
|---|---|---|---|
| 1 | (0, 0) | (0,141 ; 0,024 ; 0,148) | ❌ ratée : cube à 72,2 px, resté sur la table |
| 2 | (0 ; −5 mm) | (0,147 ; 0,034 ; 0,143) | ❌ ratée : 72,9 px |
| 3 | **(0 ; +5 mm)** | (0,159 ; 0,049 ; 0,141) | ✅ **réussie : 16,9 px**, cube soulevé (pixel 362, 93, contre 89 pour la pince) |

Après la réussite : transport au-dessus du bol, descente, lâcher, retour au repos. Le cube est visible **dans le bol** sur
l'image. Le décalage gagnant (+5 mm en y) est mémorisé (`_learned_offset`) pour les grabs suivants.

**Faux positif (grab lancé depuis le panneau) :** `grasp OK (cube 32.2 px from the gripper)`, juste sous le seuil de 35 px,
alors que le cube était resté sur la table : en perspective, un cube au sol peut apparaître juste sous la pince levée.
Le bras est allé déposer une pince vide. Le panneau n'y est pour rien (même service).
→ second critère : un cube **encore posé sur la table à moins de 6 cm de la verticale de la pince** (`RAYON_TABLE_M`,
contour à portée projeté sur le plan de la table) rend la prise **ratée**, quelle que soit la distance dans l'image. Un cube
réellement tenu (≈ 14 cm de haut) se projette à ≈ 27 cm de cette verticale avec la caméra actuelle, donc reste validé.

---

## Étape 24 — Jeu Pierre - Feuille - Ciseaux (Chifoumi) avec caméra réelle et bras simulé

**Objectif :** transformer la stack en un jeu interactif temps réel : l'humain joue contre le bras robotisé SO-ARM101 en simulation (MuJoCo / RViz). Une caméra réelle filme la main de l'utilisateur, MediaPipe classifie le geste (Pierre, Feuille, Ciseaux), et le robot joue en simultané avec un décompte synchronisé.

**Conception & Implémentation :**
1. **Détection de main (`handGesture/hand_detector.py`) :**
   - MediaPipe Hands (21 points clés / landmarks) avec backend caméra macOS (`AVFoundation`).
   - Analyse d'extension des doigts (distances euclidiennes bout de doigt / articulation intermédiaire / poignet).
   - Classification : Pierre (0 doigt levé), Feuille (4 ou 5 doigts), Ciseaux (index et majeur uniquement).
   - Lissage temporel par vote majoritaire sur les 5 dernières frames pour éliminer le sautillement.
2. **Contrôleur robotique (`handGesture/robot_chifoumi.py`) :**
   - Poses articulaires dédiées : `READY` (en garde face au joueur), `PUMP_UP` / `PUMP_DOWN` (balancements verticaux 1, 2, 3), `PIERRE` (pince serrée à 0%), `FEUILLE` (pince ouverte à 100%, poignet à plat 90°), `CISEAUX` (pince à 45%, poignet orienté horizontalement façon lames).
   - Animations de victoire (danse du robot) et de défaite (tête baissée).
   - Passerelle directe avec ROS 2 via `/joint_command` (relayé par l'API REST de `joint_panel` sur `http://localhost:8080`).
3. **Application de jeu (`handGesture/chifoumi_game.py`) :**
   - Compte à rebours animé en 3 temps (« 1... Pierre », « 2... Feuille », « 3... Ciseaux », « SHOOT ! ») avec balancements synchronisés du bras.
   - Capture et verrouillage de la pose de l'humain à l'instant précis du tir (anti-triche).
   - Arbitrage automatique, affichage HUD avec scores en direct, mode manuel (ESPACE) et mode automatique (A).
4. **Intégration et documentation :**
   - Script de démarrage rapide : `./scripts/chifoumi.sh` (ou `./scripts/chifoumi.sh --test`).
   - Guide technique et vulgarisation pour les personnes non-roboticiennes : [`docs/CHIFOUMI.md`](CHIFOUMI.md).
   - Poses et séquence `chifoumi_demo` ajoutées dans `ros2_ws/tools/joint_panel/poses.json`.
   - Suivi des évolutions dans [`CHANGELOG.md`](../CHANGELOG.md).

**Vérification :**
- `uv run python -m py_compile` sur tous les fichiers du module `handGesture/` : OK (code 0).
- `uv run handGesture/robot_chifoumi.py` : initialisation et vérification de connexion OK.
- `scripts/chifoumi.sh` exécutable et testé.

