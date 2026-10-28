#!/usr/bin/env bash
# Démarre / redémarre les conteneurs Docker du projet et configure X11 pour RViz.
#
# Usage: ./run.sh [commande]
#   start    (défaut) installe les prérequis si besoin, build et lance control + viz
#   restart  redémarre les conteneurs (sans rebuild)
#   rebuild  rebuild les images puis relance
#   stop     arrête les conteneurs
#   rviz     ouvre RViz dans le conteneur control
#   shell    ouvre un shell ROS dans le conteneur control
#   logs     suit les logs des conteneurs
#   setup    installe uniquement les prérequis
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$ROOT_DIR/docker"
SERVICES=(control viz)

info() { printf '\033[1;34m[run]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[run]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[run]\033[0m %s\n' "$*" >&2; exit 1; }

SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"

apt_install() {
  command -v apt-get >/dev/null || die "apt-get introuvable : installez manuellement : $*"
  info "Installation : $*"
  $SUDO apt-get update -qq
  $SUDO apt-get install -y --no-install-recommends "$@"
}

install_prereqs() {
  local pkgs=()
  command -v python3 >/dev/null || pkgs+=(python3)
  python3 -m pip --version >/dev/null 2>&1 || pkgs+=(python3-pip)
  python3 -c 'import venv, ensurepip' >/dev/null 2>&1 || pkgs+=(python3-venv)
  command -v xhost >/dev/null || pkgs+=(x11-xserver-utils)
  command -v docker >/dev/null || pkgs+=(docker.io)
  if ! docker compose version >/dev/null 2>&1; then
    pkgs+=(docker-compose-v2)
  fi
  [ ${#pkgs[@]} -gt 0 ] && apt_install "${pkgs[@]}"

  # Démon Docker
  if ! $SUDO docker info >/dev/null 2>&1; then
    info "Démarrage du service Docker"
    $SUDO systemctl enable --now docker
  fi

  # Groupe docker pour éviter sudo
  if [ "$(id -u)" -ne 0 ] && ! id -nG "$USER" | grep -qw docker; then
    $SUDO usermod -aG docker "$USER"
    warn "Utilisateur ajouté au groupe docker : reconnectez-vous (ou 'newgrp docker') pour ne plus utiliser sudo."
  fi
}

# Utilise sudo pour docker tant que le groupe n'est pas actif dans la session
docker_cmd() {
  if docker info >/dev/null 2>&1; then docker "$@"; else $SUDO docker "$@"; fi
}

compose() {
  (cd "$DOCKER_DIR" && docker_cmd compose "$@")
}

setup_env() {
  if [ ! -f "$DOCKER_DIR/.env" ]; then
    cp "$DOCKER_DIR/.env.example" "$DOCKER_DIR/.env"
    info "docker/.env créé depuis .env.example (USE_SIM=true par défaut, à éditer pour le vrai bras)"
  fi
}

setup_x11() {
  if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
    warn "DISPLAY non défini, utilisation de $DISPLAY"
  fi
  if xhost +local:docker >/dev/null 2>&1; then
    info "Accès X11 autorisé pour Docker (DISPLAY=$DISPLAY)"
  else
    warn "xhost a échoué : RViz ne pourra pas s'afficher (session graphique active ?)"
  fi
}

start() {
  install_prereqs
  setup_env
  setup_x11
  compose up -d --build "${SERVICES[@]}"
  info "Conteneurs lancés. RViz : ./run.sh rviz   Logs : ./run.sh logs"
}

restart() {
  setup_x11
  compose restart "${SERVICES[@]}"
  info "Conteneurs redémarrés"
}

rebuild() {
  setup_x11
  compose down
  compose build
  compose up -d "${SERVICES[@]}"
}

rviz() {
  setup_x11
  compose exec control bash -lc "source /ros2_ws/install/setup.bash && rviz2 -d /ros2_ws/so101.rviz"
}

case "${1:-start}" in
  start)   start ;;
  restart) restart ;;
  rebuild) rebuild ;;
  stop)    compose down ;;
  rviz)    rviz ;;
  shell)   compose exec control bash -lc "source /ros2_ws/install/setup.bash 2>/dev/null; exec bash" ;;
  logs)    compose logs -f "${SERVICES[@]}" ;;
  setup)   install_prereqs; setup_env ;;
  *)       sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 1 ;;
esac
