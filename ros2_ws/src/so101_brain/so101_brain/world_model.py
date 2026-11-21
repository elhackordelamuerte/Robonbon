#!/usr/bin/env python3
"""world_model.py — prédit ce que donnent un ou plusieurs coups d'échecs joués par le bras.

    ros2 run so101_brain world_model b2b4 d4c6             # état lu sur /chess/state
    ros2 run so101_brain world_model b2b4 --state /runs/scene/latest.json
    ros2 run so101_brain world_model b2b4 --play           # rejoue le mouvement sur le bras (/joint_command)

Sans --play, rien ne bouge : c'est une prédiction. --play envoie les états prédits du bras sur
/joint_command, comme le brain : le bras de la simulation (USE_SIM=true) les suit dans RViz.
Avec USE_SIM=false c'est le vrai bras qui bouge.

Le plateau prédit et le trajet de la pince sont publiés sur /chess/predicted (affichés dans
RViz par chess_markers).

Modèle volontairement simple :
  - état   : pièces (type, position, couchée ou non) + articulations du bras
  - action : coup UCI. Le bout de pince suit au-dessus de la pièce -> descente -> prise ->
             montée -> au-dessus de la case d'arrivée -> descente -> lâcher -> montée.
             Chaque point passe par l'IK (kinematics.py) : on obtient les états du bras.
  - pince  : deux cylindres verticaux autour de l'axe de la pièce saisie. Les doigts (du bout
             de pince à 4,5 cm au-dessus), de rayon « socle de la pièce + 8 mm », puis le corps
             de la pince (rayon 3 cm). La pièce portée est un cylindre sous la pince.
  - chute  : une autre pièce touchée à la hauteur z bascule si z > rayon_socle / µ (elle pivote
             sur le bord de son socle plutôt que de glisser), sinon elle est poussée.
Chaque coup part du plateau prédit par le coup précédent.
"""
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from so101_brain.kinematics import ARM_JOINTS, SO101

FILES = "abcdefgh"
MU = 0.35                    # frottement PLA / papier
FINGER_H = 0.045             # hauteur des doigts au-dessus du bout de pince
FINGER_BELOW = 0.007         # le doigt fixe dépasse sous le bout de pince
FINGER_CLEARANCE = 0.008     # rayon des doigts = rayon du socle + jeu
BODY_R = 0.03                # rayon du corps de la pince
SAFE = 0.02                  # marge au-dessus de la plus haute pièce pendant le transport
STEP = 0.005                 # pas de la trajectoire (m)
KNOCKED = 0.005              # déplacement à partir duquel une pièce est « bousculée »
GRAVEYARD = np.array([0.05, -0.20])  # où poser les pièces prises (repère du bras)
GRIP_OPEN, GRIP_CLOSED = 60.0, 0.0   # pince en % (convention /joint_command)


@dataclass
class Piece:
    id: str
    type: str
    color: str
    xy: np.ndarray           # repère du bras
    height: float
    base_r: float
    fallen: bool = False
    captured: bool = False


@dataclass
class MoveResult:
    move: str
    ok: bool = False
    fallen: list = field(default_factory=list)
    knocked: dict = field(default_factory=dict)
    ik_err_mm: float = 0.0
    arm_states: list = field(default_factory=list)  # q (5 articulations) à chaque pas
    tip_path: list = field(default_factory=list)    # bout de pince (x, y, z) à chaque pas
    grip: list = field(default_factory=list)        # pince (%) à chaque pas
    note: str = ""


class Board:
    """Passage repère plateau (origine coin a1, x vers h, y vers 8) <-> repère du bras."""

    def __init__(self, origin=(0.10, 0.10), yaw_deg=-90.0, square=0.025):
        self.origin, self.square = np.array(origin), square
        c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        self.R = np.array([[c, -s], [s, c]])

    def to_robot(self, xy):
        return self.origin + self.R @ np.asarray(xy, float)

    def to_board(self, xy):
        return self.R.T @ (np.asarray(xy, float) - self.origin)

    def center(self, sq):
        return self.to_robot(((FILES.index(sq[0]) + 0.5) * self.square, (int(sq[1]) - 0.5) * self.square))

    def square_of(self, xy):
        f, r = np.floor(self.to_board(xy) / self.square).astype(int)
        return f"{FILES[f]}{r + 1}" if 0 <= f < 8 and 0 <= r < 8 else None


class WorldModel:
    def __init__(self, board: Board):
        self.board = board
        self.arm = SO101()
        self.q_home, _, _ = self.arm.ik_best(np.array([0.15, 0.0, 0.17]), np.zeros(5))

    def load(self, state):
        self.board.square = state.get("square_size", self.board.square)
        return [
            Piece(p["id"], p["type"], p["color"], self.board.to_robot((p["x"], p["y"])),
                  p["height"], p["base_r"], fallen=p.get("fallen", False))
            for p in state["pieces"]
        ]

    def piece_at(self, pieces, sq):
        c = self.board.center(sq)
        near = [p for p in pieces if not p.captured and np.linalg.norm(p.xy - c) < 0.6 * self.board.square]
        return min(near, key=lambda p: np.linalg.norm(p.xy - c)) if near else None

    # ---------------------------------------------------------------- contact

    def contact(self, tip, held: Piece, other: Piece, carrying: bool):
        """Renvoie (pénétration horizontale, hauteur du contact) ou None."""
        d = np.linalg.norm(other.xy - tip[:2])
        zones = [
            (tip[2] - FINGER_BELOW, tip[2] + FINGER_H, held.base_r + FINGER_CLEARANCE),  # doigts
            (tip[2] + FINGER_H, 1.0, BODY_R),                                            # corps
        ]
        if carrying:
            bottom = tip[2] - self.grasp_z(held)
            zones.append((bottom, bottom + held.height, held.base_r))                     # pièce portée
        best = None
        for z0, z1, r in zones:
            lo, hi = max(z0, 0.0), min(z1, other.height)
            pen = r + other.base_r - d
            if lo < hi and pen > 0 and (best is None or hi > best[1]):
                best = (pen, hi)
        return best

    def apply_contacts(self, tip, held, pieces, carrying):
        for p in pieces:
            if p is held or p.fallen or p.captured:
                continue
            c = self.contact(tip, held, p, carrying)
            if c is None:
                continue
            pen, z = c
            if z > p.base_r / MU:
                p.fallen = True
            else:
                away = p.xy - tip[:2]
                p.xy = p.xy + away / (np.linalg.norm(away) + 1e-9) * pen

    # ---------------------------------------------------------------- mouvement

    @staticmethod
    def grasp_z(p: Piece):
        return max(0.5 * p.height, 0.012)

    def path(self, a, b):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / STEP)))
        return [a + (b - a) * i / n for i in range(1, n + 1)]

    def pick_and_place(self, pieces, p: Piece, dst_xy, q, res: MoveResult):
        tallest = max(x.height for x in pieces if not x.fallen and not x.captured)
        zg = self.grasp_z(p)
        z_up = tallest + zg + SAFE  # le bas de la pièce portée passe SAFE au-dessus des autres
        src = np.r_[p.xy, zg]
        dst = np.r_[dst_xy, zg + 0.001]
        tip, _ = self.arm.tip(q)
        legs = [  # (point d'arrivée, pièce portée ?)
            (np.r_[src[:2], z_up], False), (src, False), (np.r_[src[:2], z_up], True),
            (np.r_[dst[:2], z_up], True), (dst, True), (np.r_[dst[:2], z_up], False),
        ]
        for target, carrying in legs:
            for point in self.path(tip, target):
                q, err, _ = self.arm.ik(point, q)
                if err > 0.002:
                    q, err, _ = self.arm.ik_best(point, q)
                res.ik_err_mm = max(res.ik_err_mm, round(err * 1000, 1))
                res.arm_states.append([round(float(v), 4) for v in q])
                tip, _ = self.arm.tip(q)
                res.tip_path.append([round(float(v), 4) for v in tip])
                res.grip.append(GRIP_CLOSED if carrying else GRIP_OPEN)
                if carrying:
                    p.xy = tip[:2].copy()
                self.apply_contacts(tip, p, pieces, carrying)
            tip = target
        p.xy = dst_xy.copy()
        return q

    def play(self, pieces, move, q):
        res = MoveResult(move)
        src, dst = move[:2], move[2:4]
        p = self.piece_at(pieces, src)
        if p is None or p.fallen:
            res.note = f"pas de pièce debout en {src}"
            return res, q
        start = {x.id: (x.xy.copy(), x.fallen) for x in pieces}
        victim = self.piece_at(pieces, dst)
        if victim is not None and victim is not p:
            spot = GRAVEYARD + [0.0, -0.03 * sum(x.captured for x in pieces)]
            q = self.pick_and_place(pieces, victim, spot, q, res)
            victim.captured = True
            res.note = f"prise de {victim.id}"
        q = self.pick_and_place(pieces, p, self.board.center(dst), q, res)
        for x in pieces:
            if x.fallen and not start[x.id][1]:
                res.fallen.append(x.id)
            moved = np.linalg.norm(x.xy - start[x.id][0])
            if x is not p and x is not victim and not x.fallen and moved > KNOCKED:
                res.knocked[x.id] = round(moved * 1000, 1)
        res.ok = not p.fallen and self.board.square_of(p.xy) == dst
        return res, q

    def rollout(self, state, moves):
        """Suite d'états : chaque coup part du plateau laissé par le précédent."""
        pieces = self.load(state)
        q = self.q_home.copy()
        out = []
        for m in moves:
            res, q = self.play(pieces, m.lower(), q)
            out.append((res, self.dump(pieces, state)))
        return out

    def dump(self, pieces, state):
        out = {k: v for k, v in state.items() if k != "pieces"}
        out["pieces"] = []
        for p in pieces:
            b = self.board.to_board(p.xy)
            out["pieces"].append(dict(
                id=p.id, color=p.color, type=p.type, height=p.height, base_r=p.base_r,
                square=None if p.captured else self.board.square_of(p.xy),
                x=round(float(b[0]), 4), y=round(float(b[1]), 4),
                tilt_deg=90.0 if p.fallen else 0.0, fallen=p.fallen, captured=p.captured,
            ))
        return out


# ---------------------------------------------------------------------- CLI

def print_board(state):
    cells = {}
    for p in state["pieces"]:
        if p["square"]:
            ch = "N" if p["type"] == "knight" else p["type"][0].upper()
            cells[p["square"]] = ch.lower() if p["fallen"] else ch
    for r in range(8, 0, -1):
        print(f"   {r} " + " ".join(cells.get(f"{f}{r}", ".") for f in FILES))
    print("     " + " ".join(FILES) + "   (minuscule = pièce couchée)")


def ros_node():
    """Nœud ROS si rclpy est disponible (dans le conteneur), sinon None (test hors ROS)."""
    try:
        import rclpy
    except ImportError:
        return None
    rclpy.init()
    return rclpy.create_node("world_model")


def latched_qos():
    from rclpy.qos import DurabilityPolicy, QoSProfile

    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def state_from_topic(node, timeout=10.0):
    import rclpy
    from std_msgs.msg import String

    got = {}
    node.create_subscription(String, "chess/state", lambda m: got.setdefault("s", m.data), latched_qos())
    end = node.get_clock().now().nanoseconds + timeout * 1e9
    while "s" not in got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "s" not in got:
        sys.exit("Rien reçu sur /chess/state (chess_perception lancé ?). Sinon : --state fichier.json")
    return json.loads(got["s"])


def publish_prediction(node, final_state, path, board: Board):
    import rclpy
    from std_msgs.msg import String

    pub = node.create_publisher(String, "chess/predicted", latched_qos())
    yaw = float(np.degrees(np.arctan2(board.R[1, 0], board.R[0, 0])))
    msg = dict(final_state, path=path, board=dict(x=float(board.origin[0]), y=float(board.origin[1]), yaw=yaw))
    pub.publish(String(data=json.dumps(msg)))
    end = node.get_clock().now().nanoseconds + 1e9  # laisse le temps au message de partir
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def play_on_arm(node, results, hz=20.0):
    """Envoie les états prédits du bras sur /joint_command (comme le ferait le brain)."""
    from sensor_msgs.msg import JointState

    pub = node.create_publisher(JointState, "joint_command", 10)
    states = [(q, g) for r, _ in results for q, g in zip(r.arm_states, r.grip)]
    if not states:
        print("Rien à rejouer (aucun coup valide).")
        return
    time.sleep(0.5)  # le temps que le driver découvre le publisher
    for i, (q, g) in enumerate(states):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = [float(v) for v in q] + [float(g)]
        pub.publish(msg)
        time.sleep(1.5 if i == 0 else 1.0 / hz)  # 1,5 s pour rejoindre le premier point
    print(f"{len(states)} états envoyés sur /joint_command")


def board_from(state, a=None):
    """Pose du plateau : option en ligne de commande > marqueur ArUco (perception) > docker/.env."""
    pose = state.get("board_pose") or {}
    env = os.environ.get
    pick = lambda cli, key, var, default: cli if cli is not None else pose.get(key, float(env(var, default)))
    x = pick(getattr(a, "board_x", None), "x", "BOARD_X", 0.10)
    y = pick(getattr(a, "board_y", None), "y", "BOARD_Y", 0.10)
    yaw = pick(getattr(a, "board_yaw", None), "yaw", "BOARD_YAW", -90.0)
    src = "marqueur ArUco" if pose and getattr(a, "board_x", None) is None else "réglage"
    print(f"Plateau : coin a1 à x={x * 100:.1f} cm, y={y * 100:.1f} cm du bras, orientation {yaw:.0f}° ({src})")
    return Board((x, y), yaw, state.get("square_size", 0.025))


def main(args=None):
    ap = argparse.ArgumentParser(description="World model : coups d'échecs -> états + chutes")
    ap.add_argument("moves", nargs="+", help="coups UCI, ex. b2b4 d4c6")
    ap.add_argument("--state", help="état JSON (sinon lu sur /chess/state)")
    ap.add_argument("--board-x", type=float, help="coin a1 dans le repère du bras (m) ; sinon marqueur ArUco, sinon BOARD_X")
    ap.add_argument("--board-y", type=float)
    ap.add_argument("--board-yaw", type=float, help="° ; -90 : rangée 1 côté bras, colonne a à gauche")
    ap.add_argument("--out", help="écrit les états prédits (et les articulations du bras) en JSON")
    ap.add_argument("--play", action="store_true", help="rejoue le mouvement prédit sur le bras (/joint_command)")
    a = ap.parse_args(args if args is not None else [x for x in sys.argv[1:] if not x.startswith("--ros-args")])

    node = ros_node()
    if a.state:
        state = json.loads(Path(a.state).read_text())
    elif node is not None:
        state = state_from_topic(node)
    else:
        sys.exit("Hors ROS : donner l'état avec --state fichier.json")
    board = board_from(state, a)
    wm = WorldModel(board)

    start = wm.dump(wm.load(state), state)
    print(f"Plateau de départ ({len(start['pieces'])} pièces vues) :")
    for p in sorted(start["pieces"], key=lambda p: str(p["square"])):
        xy = board.to_robot((p["x"], p["y"]))
        print(f"  {p['square']}  {p['color']} {p['type']:<6}  bras : x={xy[0] * 100:5.1f} cm  y={xy[1] * 100:5.1f} cm"
              + ("  (couchée)" if p["fallen"] else ""))
    print_board(start)
    for m in a.moves:
        if wm.piece_at(wm.load(state), m[:2].lower()) is None:
            print(f"Attention : pas de pièce en {m[:2]} sur ce plateau, le coup {m} ne fera rien.")
    results = wm.rollout(state, a.moves)
    for i, (r, st) in enumerate(results, 1):
        print(f"\nCoup {i} : {r.move}   réussi : {'oui' if r.ok else 'NON'}   "
              f"pièce tombée : {'OUI' if r.fallen else 'non'}   ({len(r.arm_states)} états du bras, IK max {r.ik_err_mm} mm)")
        if r.fallen:
            print(f"  tombées    : {', '.join(r.fallen)}")
        if r.knocked:
            print(f"  bousculées : {', '.join(f'{k} ({v} mm)' for k, v in r.knocked.items())}")
        if r.note:
            print(f"  {r.note}")
        print_board(st)

    if node is not None:
        path = [pt for r, _ in results for pt in r.tip_path]
        publish_prediction(node, results[-1][1], path, board)
        if a.play:
            play_on_arm(node, results)
        node.destroy_node()
        import rclpy

        rclpy.shutdown()
        print("\nPrédiction publiée sur /chess/predicted (RViz : affichage Chess)")

    if a.out:
        Path(a.out).write_text(json.dumps([
            dict(move=r.move, ok=r.ok, fallen=r.fallen, knocked=r.knocked, note=r.note,
                 arm_states=r.arm_states, state=st) for r, st in results
        ], indent=1))
        print(f"\n-> {a.out}")


if __name__ == "__main__":
    main()
