#!/usr/bin/env bash
# Lance toute la stack : conteneurs control + viz, puis RViz dans le navigateur (noVNC).
#
#   ./scripts/start.sh            # lance tout et ouvre RViz
#   ./scripts/start.sh --build    # reconstruit l'image avant
#   ./scripts/start.sh --no-rviz  # sans RViz
#   Panneau de pilotage du bras : http://localhost:8080 (toujours lancé)
set -euo pipefail

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE=(docker compose -f "$RACINE/docker/docker-compose.yml")
CONTENEUR_RVIZ="tek5-rviz"
VOLUME_RVIZ="tek5-rviz-config"   # préférences RViz (~/.rviz2) conservées entre les lancements
RESEAU="tek5_rosnet"
CONTENEUR_PANNEAU="tek5-panel"
PORT_PANNEAU=8080
URL_PANNEAU="http://localhost:${PORT_PANNEAU}"
PORT_NOVNC=6080
URL_RVIZ="http://localhost:${PORT_NOVNC}/vnc.html?autoconnect=true&resize=scale"
DELAI_MAX_S=180

AVEC_BUILD=false
AVEC_RVIZ=true
for arg in "$@"; do
  case "$arg" in
    --build) AVEC_BUILD=true ;;
    --no-rviz) AVEC_RVIZ=false ;;
    -h|--help) sed -n 2,6p "$0"; exit 0 ;;
    *) echo "Option inconnue : $arg" >&2; exit 1 ;;
  esac
done

# Même ROS_DOMAIN_ID que le compose (docker/.env si présent, sinon 42).
ROS_DOMAIN_ID=$(grep -s -E '^ROS_DOMAIN_ID=' "$RACINE/docker/.env" | cut -d= -f2 || true)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export DISPLAY="${DISPLAY:-}"   # évite le warning du compose sur Mac

attendre_log() {  # attendre_log <commande logs> <motif> <message>
  local motif=$2 debut=$SECONDS
  until eval "$1" 2>&1 | grep -q -E "$motif"; do
    if (( SECONDS - debut > DELAI_MAX_S )); then
      echo "❌ Délai dépassé : $3" >&2; exit 1
    fi
    sleep 2
  done
}

docker info >/dev/null 2>&1 || { echo "❌ Docker n'est pas lancé (ouvrir Docker Desktop)." >&2; exit 1; }

if $AVEC_BUILD; then
  echo "🔨 Build de l'image…"
  "${COMPOSE[@]}" build
fi

echo "🚀 Démarrage de control + viz…"
"${COMPOSE[@]}" up -d control viz
echo "⏳ Attente du driver (build colcon + lancement)…"
attendre_log "${COMPOSE[*]} logs --since 5m control" "Driver node ready|Traceback" "driver_node"
if "${COMPOSE[@]}" logs --since 5m control 2>/dev/null | grep -q Traceback; then
  echo "❌ Erreur au lancement, voir : docker compose -f docker/docker-compose.yml logs control" >&2
  exit 1
fi
echo "✅ Driver prêt."

echo "🎛️  Démarrage du panneau de pilotage…"
docker rm -f "$CONTENEUR_PANNEAU" >/dev/null 2>&1 || true
docker run -d --name "$CONTENEUR_PANNEAU" --network "$RESEAU" -p "127.0.0.1:${PORT_PANNEAU}:8080" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
  -v "$RACINE/ros2_ws/tools/joint_panel:/panel" tek5/control \
  bash -c "source /opt/ros/jazzy/setup.bash && exec python3 /panel/panel.py" >/dev/null
attendre_log "docker logs $CONTENEUR_PANNEAU" "Panneau prêt|Traceback" "panneau"
echo "✅ Panneau prêt : $URL_PANNEAU"
command -v open >/dev/null && open "$URL_PANNEAU" || true

if $AVEC_RVIZ; then
  echo "🖥️  Démarrage de RViz (noVNC)…"
  docker rm -f "$CONTENEUR_RVIZ" >/dev/null 2>&1 || true
  docker run -d --name "$CONTENEUR_RVIZ" --network "$RESEAU" -p "127.0.0.1:${PORT_NOVNC}:6080" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA \
    -e DISPLAY=:1 -e QT_QPA_PLATFORM=xcb -e QT_XCB_NO_XI2=1 \
    -v "$RACINE/ros2_ws:/ros2_ws" -v "$VOLUME_RVIZ:/root/.rviz2" tek5/control bash -c '
set -e
apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends xvfb x11vnc novnc websockify fluxbox xdotool >/dev/null
Xvfb :1 -screen 0 1600x1000x24 &
sleep 2
fluxbox >/dev/null 2>&1 &
x11vnc -display :1 -forever -shared -nopw -localhost -quiet -rfbport 5900 &
websockify --web /usr/share/novnc 6080 localhost:5900 &
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
echo PRET_RVIZ
while true; do rviz2 -d /ros2_ws/so101.rviz; sleep 2; done
' >/dev/null
  echo "⏳ Installation de Xvfb/noVNC dans le conteneur (~1 min)…"
  attendre_log "docker logs $CONTENEUR_RVIZ" "PRET_RVIZ" "RViz"
  sleep 3
  echo "✅ RViz prêt : $URL_RVIZ"
  command -v open >/dev/null && open "$URL_RVIZ" || true
fi

cat <<EOF

Tout est lancé.
  Panneau de pilotage : $URL_PANNEAU

Ou à la main :
  cd docker && docker compose exec control bash
  source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
  ros2 topic pub -r 10 --times 10 /joint_command sensor_msgs/msg/JointState \\
    "{name: [shoulder_pan, elbow_flex, gripper], position: [0.6, 0.5, 90.0]}"

Config RViz : Ctrl+S dans RViz pour enregistrer (ros2_ws/so101.rviz),
enregistrée aussi automatiquement par ./scripts/stop.sh.

Pour tout arrêter : ./scripts/stop.sh
EOF
