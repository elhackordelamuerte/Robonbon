#!/usr/bin/env bash
# Tourne et monte la vidéo de démo de la phase 1 en simulation : vue RViz + overlay
# (étape en cours, essai n°, valeurs en direct, vérification, incrustation caméra, chrono).
#
#   ./scripts/video_sim.sh                 # tournage (relance toute la stack) + montage
#   ./scripts/video_sim.sh --montage-seul  # remonte la vidéo à partir des derniers rushes
#   BIAIS_Y_MM=25 NB_CUBES=2 ./scripts/video_sim.sh
#
# Scénario : une erreur de calibration caméra volontaire (BIAIS_X_MM / BIAIS_Y_MM, via
# correction_objet de la perception) fait rater le 1er essai ; le cerveau relance à 5 mm à côté,
# réussit, garde l'offset appris ; le cube suivant est pris du premier coup.
# Avec 25 mm en y (essais du 2026-11-09) : le 1er essai rate souvent, pas toujours (ça dépend de
# l'orientation du cube) ; s'il réussit, la prise est automatiquement recommencée (MAX_PRISES).
#
# Résultat : docs/soutenance/videos/pick_place_sim.mp4
# Rushes   : ros2_ws/tools/video/rushes/ (rviz.mp4, evenements.jsonl, camera/*.jpg, scene/*.jpg, overlay.ass)
# RViz montre les TF (world, base_link, pince, caméra) et le cube vu par la caméra (aides_rviz.py) ;
# la vue d'ensemble MuJoCo vient du driver (paramètre scene_view, activé seulement pendant le tournage).
#
# ⚠️ Relance toute la stack (stop.sh + start.sh) pour repartir d'un offset appris nul.
set -euo pipefail

RACINE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTILS="$RACINE/ros2_ws/tools/video"
RUSHES="$OUTILS/rushes"
RUSHES_C="/ros2_ws/tools/video/rushes"          # même dossier, vu depuis les conteneurs
SORTIE="$RACINE/docs/soutenance/videos/pick_place_sim.mp4"
CONTENEUR_VIDEO="tek5-video"
CONTENEUR_PERCEPTION="tek5-perception-video"
RESEAU="tek5_rosnet"
BIAIS_X_MM="${BIAIS_X_MM:-0}"
BIAIS_Y_MM="${BIAIS_Y_MM:-25}"
NB_CUBES="${NB_CUBES:-2}"
ECRAN=":2"
TAILLE_ECRAN="2400x1200"
ZONE_3D="1920x1080"
ORIGINE_3D="+460,62"                             # coin haut-gauche de la vue 3D de RViz
DELAI_PICK_S=300

ROS_DOMAIN_ID=$(grep -s -E '^ROS_DOMAIN_ID=' "$RACINE/docker/.env" | cut -d= -f2 || true)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
ENV_ROS=(-e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" -e FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA)
SOURCER="source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash"

ros() {  # ros <commande> : exécute une commande ROS dans le conteneur vidéo
  docker exec "$CONTENEUR_VIDEO" bash -c "$SOURCER && $1"
}

compter() {  # compter <motif> : nombre de logs du cerveau correspondant au motif
  grep -c -E "$1" "$RUSHES/evenements.jsonl" 2>/dev/null || true
}

monter() {
  echo "🎬 Montage (overlay + incrustation caméra)…"
  docker exec "$CONTENEUR_VIDEO" python3 /ros2_ws/tools/video/montage.py "$RUSHES_C"
  mkdir -p "$(dirname "$SORTIE")"
  cp "$RUSHES/pick_place_sim.mp4" "$SORTIE"
  echo "✅ Vidéo : $SORTIE"
}

demarrer_conteneur_video() {
  docker rm -f "$CONTENEUR_VIDEO" >/dev/null 2>&1 || true
  docker run -d --name "$CONTENEUR_VIDEO" --network "$RESEAU" "${ENV_ROS[@]}" \
    -e DISPLAY="$ECRAN" -e QT_QPA_PLATFORM=xcb -e QT_XCB_NO_XI2=1 \
    -v "$RACINE/ros2_ws:/ros2_ws" tek5/control bash -c "
apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends xvfb ffmpeg fonts-dejavu-core >/dev/null
Xvfb $ECRAN -screen 0 ${TAILLE_ECRAN}x24 &
sleep 2
echo PRET_VIDEO
sleep infinity" >/dev/null
  until docker logs "$CONTENEUR_VIDEO" 2>&1 | grep -q PRET_VIDEO; do sleep 2; done
}

nettoyer() {
  docker exec "$CONTENEUR_VIDEO" pkill -INT -f enregistreur.py >/dev/null 2>&1 || true
  docker exec tek5-control-1 bash -c "$SOURCER && ros2 param set /so101_driver scene_view false" >/dev/null 2>&1 || true
  docker rm -f "$CONTENEUR_PERCEPTION" >/dev/null 2>&1 || true
  docker start tek5-perception-1 >/dev/null 2>&1 || true   # perception normale (sans erreur)
}

if [ "${1:-}" = "--montage-seul" ]; then
  docker ps --format '{{.Names}}' | grep -qx "$CONTENEUR_VIDEO" || demarrer_conteneur_video
  monter
  docker rm -f "$CONTENEUR_VIDEO" >/dev/null
  exit 0
fi

docker info >/dev/null 2>&1 || { echo "❌ Docker n'est pas lancé (ouvrir Docker Desktop)." >&2; exit 1; }
trap nettoyer EXIT

echo "🔄 Relance de la stack (offset appris remis à zéro)…"
"$RACINE/scripts/stop.sh" >/dev/null
"$RACINE/scripts/start.sh" --no-rviz | grep -E "✅|❌" || true

echo "🖥️  Conteneur vidéo (Xvfb + ffmpeg)…"
demarrer_conteneur_video
rm -rf "$RUSHES" && mkdir -p "$RUSHES"
# la découverte DDS d'un conteneur tout neuf prend quelques secondes
for _ in $(seq 1 30); do
  ros "ros2 node list" 2>/dev/null | grep -qx /so101_driver && break
  sleep 2
done

echo "🎯 Perception avec erreur de calibration simulée ($BIAIS_X_MM ; $BIAIS_Y_MM) mm…"
ros "ros2 param get /so101_driver cam_T" | python3 -c "
import re, sys
v = [float(x) for x in re.findall(r'-?[\d.]+(?:e-?\d+)?', sys.stdin.read().split(':', 1)[1])]
assert len(v) == 16, v
print('cam_T: ' + str(v))
print('correction_objet: [%s, %s]' % ($BIAIS_X_MM / 1000, $BIAIS_Y_MM / 1000))
" > "$RUSHES/pose_camera_biais.yaml"
echo "{\"biais_mm\": [$BIAIS_X_MM, $BIAIS_Y_MM]}" > "$RUSHES/parametres.json"
docker stop tek5-perception-1 >/dev/null
docker rm -f "$CONTENEUR_PERCEPTION" >/dev/null 2>&1 || true
docker run -d --name "$CONTENEUR_PERCEPTION" --network "$RESEAU" "${ENV_ROS[@]}" \
  -v "$RACINE/ros2_ws:/ros2_ws" tek5/perception bash -c "$SOURCER && exec ros2 run so101_perception perception \
  --ros-args -p camera_ns:=/external_cam -p convention_mujoco:=true -p cam_T_file:=$RUSHES_C/pose_camera_biais.yaml" >/dev/null

echo "🎥 Vue d'ensemble de la scène + TF caméra et cube pour RViz…"
ros "ros2 param set /so101_driver scene_view true" >/dev/null
docker exec -d "$CONTENEUR_VIDEO" bash -c "$SOURCER && exec python3 /ros2_ws/tools/video/aides_rviz.py $RUSHES_C/pose_camera_biais.yaml > /tmp/aides.log 2>&1"

echo "🖥️  RViz (vue vidéo)…"
docker exec -d "$CONTENEUR_VIDEO" bash -c "$SOURCER && exec rviz2 -d /ros2_ws/tools/video/so101_video.rviz > /tmp/rviz.log 2>&1"
sleep 10

placer_cube() {  # tire une position de cube jusqu'à ce que la perception le voie
  for _ in $(seq 1 10); do
    ros "timeout 20 ros2 service call /reset_object std_srvs/srv/Trigger" >/dev/null 2>&1 || true
    sleep 3
    if ros "timeout 4 ros2 topic echo --once /object_position" >/dev/null 2>&1; then return 0; fi
  done
  echo "❌ La perception ne voit pas le cube après 10 tirages." >&2; exit 1
}

pick_and_place() {  # lance un pick & place et attend sa fin
  local avant
  avant=$(compter 'pick & place: done|giving up')
  ros "timeout 20 ros2 service call /pick_and_place std_srvs/srv/Trigger" | grep -o "message=.*" || true
  local debut=$SECONDS
  until [ "$(compter 'pick & place: done|giving up')" -gt "$avant" ]; do
    (( SECONDS - debut > DELAI_PICK_S )) && { echo "❌ pick & place trop long" >&2; exit 1; }
    sleep 1
  done
}

demarrer_capture() {
  rm -rf "$RUSHES/camera" "$RUSHES/evenements.jsonl" "$RUSHES/rviz.mp4" "$RUSHES/debut.txt"
  docker exec -d "$CONTENEUR_VIDEO" bash -c "$SOURCER && exec python3 /ros2_ws/tools/video/enregistreur.py $RUSHES_C"
  sleep 2
  docker exec -d "$CONTENEUR_VIDEO" bash -c "date +%s.%N > $RUSHES_C/debut.txt && exec ffmpeg -loglevel error -y \
    -f x11grab -draw_mouse 0 -framerate 30 -video_size $ZONE_3D -i $ECRAN$ORIGINE_3D \
    -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p $RUSHES_C/rviz.mp4 > /tmp/ffmpeg.log 2>&1"
}

arreter_capture() {
  docker exec "$CONTENEUR_VIDEO" pkill -INT ffmpeg || true
  docker exec "$CONTENEUR_VIDEO" pkill -INT -f enregistreur.py || true
  sleep 3
}

# L'erreur de calibration ne fait pas rater à coup sûr : selon l'orientation (aléatoire) du cube,
# les mâchoires peuvent la rattraper en se fermant. Si le 1er cube est pris du premier coup,
# la prise est jetée et recommencée : l'offset appris reste nul (essai 1 = offset appris + 0).
MAX_PRISES=${MAX_PRISES:-6}
for prise in $(seq 1 "$MAX_PRISES"); do
  placer_cube
  echo "⏺️  Tournage, prise ${prise}/${MAX_PRISES}…"
  demarrer_capture
  sleep 6   # titre
  echo "   cube 1/$NB_CUBES"
  pick_and_place
  if [ "$(compter 'grasp FAILED')" -gt 0 ]; then break; fi
  arreter_capture
  echo "   ↻ 1er cube pris du premier coup (pas de relance à montrer) : nouvelle prise"
  [ "$prise" -eq "$MAX_PRISES" ] && { echo "❌ Aucun raté en $MAX_PRISES prises : augmenter BIAIS_Y_MM" >&2; exit 1; }
done
for n in $(seq 2 "$NB_CUBES"); do
  echo "   cube $n/$NB_CUBES"
  placer_cube; sleep 2
  pick_and_place
done
sleep 7   # carton de fin
arreter_capture

monter
docker rm -f "$CONTENEUR_VIDEO" >/dev/null
echo "ℹ️  Perception normale relancée ; le cerveau garde l'offset appris de la vidéo (stop.sh + start.sh pour repartir de zéro)."
