# Piloter le vrai bras SO-ARM101 depuis un Mac

Ce guide permet de commander le **bras réel** avec la stack ROS 2 existante (panneau, brain, `/joint_command`),
en remplaçant simplement la simulation par le vrai robot (`USE_SIM=false`).

> ⚠️ **Sécurité :** un vrai bras peut se blesser, ou blesser quelqu'un. Dégager l'espace de travail, garder une main près
> de l'alimentation, et faire les premiers essais avec le panneau en **envoi non direct** (bouton *Envoyer*).

> 💡 **Le panneau (http://localhost:8080) guide tout le passage**, carte *Simulation ↔ Bras réel* : mode actuel
> (lu dans le driver), état de la calibration et du pont série, étapes numérotées avec les commandes prêtes à copier
> (remplies avec le port saisi), consignes de sécurité et retour en simulation.

## 1. Pourquoi un pont série ?

Docker Desktop (macOS) **ne donne pas accès aux ports USB** depuis les conteneurs : le driver, dans `control`, ne voit pas
`/dev/tty.usbmodem…`. On relaie donc le port série par le réseau local :

```
bras (USB) ─► scripts/serial_bridge.py ─TCP 127.0.0.1:8765─► socat (conteneur control) ─► /dev/ttyBRIDGE ─► driver_node
   Mac            pyserial, 1 Mbaud        host.docker.internal        port série virtuel        USE_SIM=false
```

- Latence mesurée de l'aller-retour (écho, sans bras) : **0,5 ms** en médiane, 11 ms au pire. La boucle du driver tourne à 50 Hz (20 ms).
- **Port 8765, pas 7000 :** sur macOS, le récepteur AirPlay (`ControlCenter`) écoute déjà sur 5000 et 7000.
- Sous Linux, pas besoin de pont : le compose donne déjà accès à `/dev/ttyACM*` (`device_cgroup_rules`).

## 2. Trouver le port et calibrer (une fois par bras)

La calibration utilise **l'outil officiel LeRobot** (Hugging Face), exécuté sur le Mac avec un accès USB direct.
`uvx` l'installe dans un environnement isolé (lourd la première fois, car il embarque PyTorch) :

```bash
# Brancher le bras (USB + alimentation), puis :
uvx --from 'lerobot[feetech]' lerobot-find-port          # débrancher le câble quand demandé → affiche le port

uvx --from 'lerobot[feetech]' lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431551 \
    --robot.id=arm \
    --robot.calibration_dir=./calib                      # → calib/arm.json
```

Procédure interactive (le couple est coupé, le bras se manipule à la main) :
1. placer **toutes les articulations au milieu** de leur course, puis Entrée ;
2. bouger **chaque articulation sur toute sa course** (sauf `wrist_roll`, considéré comme un tour complet), puis Entrée.

`calib/arm.json` contient, pour chaque moteur, `homing_offset`, `range_min` et `range_max` (en ticks). Il est monté dans le
conteneur sur `/calibration/arm.json` (`CALIBRATION_FILE`). Le driver vérifie au démarrage que le fichier correspond aux
moteurs (`is_calibrated`), et refuse sinon.

> Référence : [doc SO-101 de LeRobot](https://huggingface.co/docs/lerobot/so101) (section *Calibrate*),
> [DeepWiki — Calibration and setup](https://deepwiki.com/huggingface/lerobot/6.4-calibration-and-setup).

## 3. Lancer

```bash
uv run scripts/serial_bridge.py          # terminal 1 : laisser tourner (port détecté auto, sinon --port …)
USE_SIM=false ./scripts/start.sh         # terminal 2
```

`start.sh` en mode bras réel :
- utilise `SERIAL_BRIDGE=tcp:host.docker.internal:8765` et `CALIBRATION_FILE=calib/arm.json` par défaut ;
- **refuse de démarrer** si la calibration manque, ou si le pont n'écoute pas (message avec la commande à lancer).

Le bras se commande ensuite exactement comme en simulation : panneau http://localhost:8080 (sliders, poses, séquences),
`ros2 topic pub /joint_command …`, ou les services du brain (`/go_to_rest`, `/set_joint_positions`, `/go_to_target`).

Pour revenir à la simulation : `./scripts/stop.sh && ./scripts/start.sh`.

## 4. Garde-fous ajoutés

| Garde-fou | Où | Effet |
|---|---|---|
| Limite de vitesse | driver, `max_step_rad` = 0,05 rad et `max_step_gripper_pct` = 2 % | LeRobot `max_relative_target` : écart consigne/position limité à chaque pas de 20 ms → **2,5 rad/s max** (≈ 143°/s), pince 100 %/s |
| Pas de mouvement au démarrage | brain, `rest_on_start:=false` si `USE_SIM=false` | le bras ne part pas tout seul vers la pose de repos |
| Pose tenue au démarrage | driver | la première consigne est la position mesurée : pas de saut vers 0 |
| Bornes des joints | driver (`JOINT_LIMITS`, URDF) | consignes hors limites ramenées aux limites |
| Couple coupé à l'arrêt | `destroy_node()` → `disconnect()` | `./scripts/stop.sh` relâche le bras (il retombe : **le soutenir**) |
| Calibration vérifiée | driver | refus si `calib/arm.json` ne correspond pas aux moteurs |

Ralentir encore (premiers essais) : `ros2 param` ne suffit pas, car la limite est fixée à la connexion. Il faut modifier
`max_step_rad` dans `driver.launch.py` (ou dans les paramètres du nœud), puis relancer.

## 5. Le couple : choix du dépôt du cours

La version du créateur du dépôt coupe le couple juste après la connexion (`robot.bus.disable_torque()`, « *Comment out this
line to control the robot.* ») : le bras est alors libre à la main, mais **non pilotable**. Pour pouvoir le commander, ce bloc
est **commenté** dans notre `driver_node.py`, comme le dépôt l'indique. Pour retrouver le mode « bras libre » (par exemple
pour lire `/joint_states` en bougeant le bras à la main), il suffit de le décommenter.

## 6. Dépannage

| Symptôme | Cause probable / solution |
|---|---|
| `❌ Port TCP 8765 déjà utilisé` | un `serial_bridge` tourne déjà : `pkill -f serial_bridge.py` |
| `❌ 0 port(s) USB-série trouvé(s)` | bras non branché ou non alimenté ; `uv run scripts/serial_bridge.py --list` |
| driver : `motor check failed` | mauvais port, alimentation coupée, ou pont relié à autre chose qu'au bras |
| driver : `does not match the motors` | `calib/arm.json` ne correspond pas à ce bras : relancer `lerobot-calibrate` |
| le bras ne bouge pas, `/joint_states` change à la main | couple coupé : vérifier que `disable_torque()` est bien commenté |
| `lerobot-calibrate` ne trouve pas le port | fermer `serial_bridge.py` : un port série ne peut être ouvert que par un programme à la fois |
