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
