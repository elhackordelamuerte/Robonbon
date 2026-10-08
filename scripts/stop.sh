#!/usr/bin/env bash
# Arrête toute la stack : conteneur RViz (noVNC) puis conteneurs du compose.
#
#   ./scripts/stop.sh
set -euo pipefail

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTENEUR_RVIZ="tek5-rviz"
export DISPLAY="${DISPLAY:-}"   # évite le warning du compose sur Mac

docker info >/dev/null 2>&1 || { echo "❌ Docker n'est pas lancé." >&2; exit 1; }

# Enregistre la config RViz (Ctrl+S -> ros2_ws/so101.rviz, monté depuis l'hôte)
# avant de supprimer le conteneur, sinon les modifications non enregistrées sont perdues.
sauver_config_rviz() {
  docker exec "$CONTENEUR_RVIZ" bash -c '
    export DISPLAY=:1
    command -v xdotool >/dev/null || exit 1
    fenetre=$(xdotool search --onlyvisible --name "RViz" | head -1)
    [ -n "$fenetre" ] || exit 1
    xdotool windowactivate --sync "$fenetre" key --clearmodifiers ctrl+s
    sleep 1' >/dev/null 2>&1
}

if docker ps --format '{{.Names}}' | grep -qx "$CONTENEUR_RVIZ"; then
  if sauver_config_rviz; then
    echo "💾 Config RViz enregistrée (ros2_ws/so101.rviz)."
  else
    echo "⚠️  Config RViz non enregistrée automatiquement (RViz fermé ?)." >&2
  fi
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTENEUR_RVIZ"; then
  echo "🛑 Arrêt de RViz…"
  docker rm -f "$CONTENEUR_RVIZ" >/dev/null
fi

# Le conteneur RViz est supprimé avant : il utilise le réseau tek5_rosnet,
# qui bloquerait sinon sa suppression par `compose down`.
echo "🛑 Arrêt de control + viz (le driver déconnecte proprement le bras)…"
docker compose -f "$RACINE/docker/docker-compose.yml" down

echo "✅ Tout est arrêté."
