# Tek5 — SO-ARM101 pick & place

Simulation, IK, perception and pick & place on the SO-ARM101 arm, with ROS2 and Docker.

- Project spec: [`docs/SUJET.md`](docs/SUJET.md)
- Setup and first steps: see the bootstrap slides.

## World model (échecs)

Caméra -> pièces (`so101_perception`, `/chess/state`) -> coups simulés et risque de chute
(`ros2 run so101_brain world_model b2b4 d4c6`). Détails : [`docs/WORLD_MODEL.md`](docs/WORLD_MODEL.md).
