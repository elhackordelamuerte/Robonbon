#!/usr/bin/env bash
# Lance le jeu Pierre - Feuille - Ciseaux (Chifoumi) contre le SO-ARM101.
#
#   ./scripts/chifoumi.sh            # Lance le jeu avec la webcam par défaut (index 0)
#   ./scripts/chifoumi.sh --index 1  # Spécifie un autre index de caméra
#   ./scripts/chifoumi.sh --test     # Test uniquement de la détection de main
#
set -euo pipefail

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$RACINE"
export PYTHONPATH="$RACINE:${PYTHONPATH:-}"

# Vérification de uv
command -v uv >/dev/null 2>&1 || {
  echo "❌ 'uv' n'est pas installé. Installez-le avec : curl -LsSf https://astral.sh/uv/install.sh | sh" >&2
  exit 1
}

# Vérifier si le panneau ROS 2 tourne
if curl -s -m 1 http://localhost:8080/api/etat >/dev/null 2>&1; then
  echo "✅ Bras robotisé / Panneau ROS 2 détecté sur http://localhost:8080"
else
  echo "ℹ️  Le panneau ROS 2 n'est pas actif."
  echo "   Si vous souhaitez voir le bras bouger en simulation :"
  echo "   1. Dans un autre terminal : ./scripts/start.sh"
  echo "   2. Ouvrez RViz sur http://localhost:6080"
  echo "   Le jeu continue en attendant le robot..."
fi

if [[ "${1:-}" == "--test" ]]; then
  echo "🖐️ Lancement du test de détection de main..."
  exec uv run "$RACINE/handGesture/hand_detector.py" "${@:2}"
else
  echo "🎮 Lancement de Chifoumi..."
  exec uv run "$RACINE/handGesture/chifoumi_game.py" "$@"
fi
