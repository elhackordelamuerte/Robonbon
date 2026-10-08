# Lancer le projet et RViz sur macOS

Ce tuto remplace la slide « Setup » pour les Mac. Il a été testé sur un Mac Apple Silicon (arm64) avec Docker Desktop.

> **Pourquoi c'est différent sur Mac ?**
> Sur Linux, RViz s'affiche via X11 (`xhost +local:docker` + socket `/tmp/.X11-unix`).
> Sur Mac, Docker tourne dans une VM : le socket X11 n'est pas partagé, et XQuartz
> ne fournit pas un OpenGL suffisant pour RViz (erreur `OpenGL 1.5 is not supported`).
> On fait donc tourner RViz dans un **écran virtuel (Xvfb)** à l'intérieur d'un conteneur,
> et on l'affiche **dans le navigateur via noVNC**. Pas besoin d'XQuartz.

## Prérequis

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) installé et lancé
- Un navigateur web
- Le dépôt cloné

## Démarrage rapide (scripts)

Depuis la racine du dépôt :

```bash
./scripts/start.sh            # lance control + viz + RViz, puis ouvre le navigateur
./scripts/start.sh --build    # idem, en reconstruisant l'image d'abord
./scripts/start.sh --no-rviz  # sans RViz
./scripts/stop.sh             # arrête tout
```

La config RViz est enregistrée dans `ros2_ws/so101.rviz` : avec **Ctrl+S** dans RViz, et automatiquement par `stop.sh`.

Les sections suivantes détaillent ce que font ces scripts, étape par étape.

## 1. Construire l'image (une seule fois)

```bash
cd docker
docker compose build
```

## 2. Lancer la stack ROS 2

Toujours depuis `docker/` :

```bash
docker compose up -d control viz
docker compose logs -f control     # attendre "Brain node ready." puis Ctrl-C
```

Par défaut, le bras est en **simulation** (`USE_SIM=true`). Pour changer la config :
`cp .env.example .env` puis éditer `.env`.

## 3. Lancer RViz (dans le navigateur)

Depuis la **racine du dépôt** :

```bash
docker run -d --name tek5-rviz --network tek5_rosnet -p 127.0.0.1:6080:6080 \
  -e ROS_DOMAIN_ID=42 -e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  -e DISPLAY=:1 -e QT_QPA_PLATFORM=xcb -e QT_XCB_NO_XI2=1 \
  -v "$PWD/ros2_ws:/ros2_ws" tek5/control bash -c '
set -e
apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends xvfb x11vnc novnc websockify fluxbox >/dev/null
Xvfb :1 -screen 0 1600x1000x24 &
sleep 2
fluxbox >/dev/null 2>&1 &
x11vnc -display :1 -forever -shared -nopw -localhost -quiet -rfbport 5900 &
websockify --web /usr/share/novnc 6080 localhost:5900 &
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
echo PRET_RVIZ
while true; do rviz2 -d /ros2_ws/so101.rviz; sleep 2; done
'
```

Attendre ~1 minute (installation de Xvfb/noVNC au démarrage), jusqu'à voir `PRET_RVIZ` :

```bash
docker logs -f tek5-rviz
```

Puis ouvrir :

```bash
open "http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
```

> Si tu as changé `ROS_DOMAIN_ID` dans `.env`, mets la même valeur dans la commande `docker run`.

## 4. Ouvrir un shell dans le conteneur control

```bash
cd docker
docker compose exec control bash

# dans chaque nouveau shell
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# vérification
ros2 topic list
ros2 topic hz /joint_states
```

> Tant que `driver_node.py` n'est pas implémenté (TODO dans
> `ros2_ws/src/so101_driver/so101_driver/driver_node.py`), `/joint_states` n'est pas
> publié : c'est normal, le bras restera immobile dans RViz.

## Commandes utiles

| Action | Commande |
|---|---|
| Relancer RViz | `docker restart tek5-rviz` |
| Logs RViz | `docker logs -f tek5-rviz` |
| Arrêter RViz | `docker rm -f tek5-rviz` |
| Arrêter la stack | `cd docker && docker compose down` |

## Dépannage

| Symptôme | Solution |
|---|---|
| `network tek5_rosnet not found` | Lancer d'abord l'étape 2 (`docker compose up -d control viz`). |
| `Conflict. The container name "/tek5-rviz" is already in use` | `docker rm -f tek5-rviz` puis relancer l'étape 3. |
| Impossible de cliquer dans RViz | Vérifier que la commande contient `fluxbox` et `-e QT_XCB_NO_XI2=1` (sans gestionnaire de fenêtres, Qt ignore les clics VNC), puis recréer le conteneur. |
| noVNC figé / déconnecté | Recharger la page, sinon `docker restart tek5-rviz`. |
| Page noVNC inaccessible | Attendre `PRET_RVIZ` dans `docker logs tek5-rviz`. |
| RViz ne voit aucun topic | Vérifier que `ROS_DOMAIN_ID` est identique partout (42 par défaut). |
| Bras USB réel non détecté | Docker Desktop sur Mac ne fait pas de passthrough USB : utiliser la simulation, ou une VM Linux avec passthrough USB. |
