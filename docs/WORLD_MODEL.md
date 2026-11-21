# World model : le SO-101 joue aux échecs

On donne un ou plusieurs coups. Le world model prédit les mouvements du bras, l'état du plateau
après chaque coup et si une pièce tombe.

## Où est le code

| Paquet | Fichier | Rôle |
|---|---|---|
| `so101_perception` | `chess_vision.py` | photo -> état du plateau (sans ROS) |
| `so101_perception` | `chess_perception_node.py` | nœud : caméra -> YOLO -> publie `/chess/state` (vote sur 5 images) |
| `so101_brain` | `kinematics.py` | FK / IK du SO-101 lues dans l'URDF de `so101_description` |
| `so101_brain` | `world_model.py` | `ros2 run so101_brain world_model <coups>`, publie `/chess/predicted` |
| `so101_brain` | `chess_markers.py` | RViz : plateau, pièces vues, pièces prédites, trajet de la pince (`/chess/markers`) |

`/chess/state` : `std_msgs/String` en JSON, x/y des pièces dans le repère plateau (origine au
coin extérieur de a1, x vers h, y vers 8), avec hauteur et rayon de socle de chaque pièce.

Perception : seules les cases où une pièce est vue dans au moins 60 % des 5 dernières images
sont publiées (une pièce par case, type le plus fréquent). Le plateau est la médiane des
dernières détections réussies : une main qui le masque ne change rien.

RViz (`./run.sh rviz`, affichage « Chess ») : pièces vues en blanc/noir, prédites en vert
(rouge si tombées), trajet de la pince en bleu.

## Le modèle

- **État** : pièces (type, position, couchée ou non) + articulations du bras.
- **Action** : coup UCI (`b2b4`). Le bout de pince va au-dessus de la pièce, descend, la prend,
  remonte, va au-dessus de la case d'arrivée, descend, lâche, remonte. Chaque point (tous les
  5 mm) passe par l'IK : ce sont les états du bras.
- **Pince** : deux cylindres verticaux autour de la pièce saisie. Les doigts (rayon du socle
  + 8 mm, jusqu'à 4,5 cm au-dessus du bout), puis le corps de la pince (rayon 3 cm). La pièce
  portée est un cylindre sous la pince.
- **Chute** : une pièce touchée à la hauteur `z` bascule si `z > rayon_socle / µ` (µ = 0,35) :
  elle pivote sur le bord de son socle avant de glisser. Sinon elle est poussée.
- Chaque coup part du plateau prédit par le coup précédent. Une pièce prise est posée hors du
  plateau.

Le modèle est pessimiste pour les cases voisines : saisir ou poser une pièce à côté d'une autre
(cases adjacentes, 25 mm) la touche presque toujours.

## Lancer

1. Activer la perception dans `docker/.env` :
   ```
   COMPOSE_PROFILES=perception
   PERCEPTION_SOURCE=/dev/video0          # ou une photo : /images/image2.jpg
   ```
   puis `./run.sh`. Le premier lancement installe ultralytics dans le volume `yolo_pkgs`
   (quelques minutes). L'état et l'image annotée sont écrits dans `runs/scene/latest.*`.

2. Prédire :
   ```bash
   ./run.sh shell
   ros2 run so101_brain world_model b2b4 d4c6 h6b6
   ros2 run so101_brain world_model b2b4 --state /ros2_ws/state.json --out /ros2_ws/pred.json
   ```
   Options : `--board-x --board-y --board-yaw` (position du coin a1 dans le repère du bras ;
   par défaut 10 cm devant, rangée 1 côté bras), `--out` (états du plateau et articulations).

Test hors Docker, sur une photo avec les détections YOLO déjà sauvées :
```bash
cd ros2_ws/src/so101_perception
python3 -m so101_perception.chess_vision ../../../images/image2.jpg \
  --labels ../../../runs/detect/predict-10/labels/image2.txt --data ../../../dataset/data.yaml --out /tmp/scene
cd ../so101_brain
python3 -m so101_brain.world_model b2b4 d4c6 --state /tmp/scene/image2.json
```

## Position du plateau par rapport au bras (automatique)

Un marqueur ArUco scotché sur la table donne la position du plateau à chaque image : la feuille
peut être posée n'importe où devant le bras, il n'y a plus rien à mesurer quand on la déplace.

1. Générer la feuille du marqueur et l'imprimer à 100 % (vérifier 8 cm à la règle) :
   ```bash
   PYTHONPATH=ros2_ws/src/so101_perception python3 -m so101_perception.chess_vision --marker-sheet aruco.png
   ```
2. Le découper et le scotcher sur la table, à plat, devant le bras, flèche vers l'avant du bras,
   à quelques centimètres de la feuille de l'échiquier, dans le champ de la caméra.
3. Une seule mesure : le centre du marqueur depuis l'axe du moteur de la base. Dans docker/.env :
   `MARKER_X` = distance vers l'avant + 0,039, `MARKER_Y` = distance vers la gauche (m).
   Si la flèche n'est pas vers l'avant : `MARKER_YAW` (90 = vers la gauche du bras).

La perception publie alors `board_pose` dans `/chess/state` ; `world_model` et RViz l'utilisent.
a1 est choisi automatiquement : la case foncée de coin la plus proche du bras.
Sans marqueur visible, ce sont `BOARD_X`, `BOARD_Y`, `BOARD_YAW` de docker/.env qui servent.

## Réglages à mesurer

- `SQUARE_SIZE` et `PIECE_SPECS` dans `chess_vision.py` (case 25 mm, hauteurs et socles
  estimés sur les photos).
- Position du plateau par rapport au bras (`--board-x/y/yaw`).
- a1 est pris au coin foncé le plus bas dans l'image (paramètre `flip` pour tourner de 180°).
