"""Cinématique du SO-101 lue dans l'URDF de so101_description (numpy seulement).

    arm = SO101()
    tip, approach = arm.tip(q)            # bout de pince (gripper_frame_link) et direction de la pince
    q, err, angle = arm.ik_best(xyz, q0)  # IK : position d'abord, pince verticale si possible

Repère : world = base_link (comme viz.launch.py), x devant, z en haut. q = 5 articulations du
bras en radians (ordre de /joint_states, sans la pince).
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def default_urdf() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
    except Exception:  # hors ROS : arbre source
        return Path(__file__).resolve().parents[2] / "so101_description" / "urdf" / "so101.urdf"


ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
IK_JOINTS = ARM_JOINTS[:4]  # wrist_roll garde sa valeur (pièces rondes)
TIP = "gripper_frame_link"
DOWN = np.array([0.0, 0.0, -1.0])


# --------------------------------------------------------------------------- géométrie

def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def rot_axis(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


def _vec(s, default="0 0 0"):
    return np.array([float(v) for v in (s or default).split()])


# --------------------------------------------------------------------------- modèle

class SO101:
    def __init__(self, urdf=None):
        root = ET.parse(urdf or default_urdf()).getroot()
        self.joints = {}
        self.children = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            lim = j.find("limit")
            info = dict(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                origin=transform(_vec(o.get("xyz") if o is not None else None), _vec(o.get("rpy") if o is not None else None)),
                axis=_vec(j.find("axis").get("xyz") if j.find("axis") is not None else None, "0 0 1"),
                lower=float(lim.get("lower")) if lim is not None else 0.0,
                upper=float(lim.get("upper")) if lim is not None else 0.0,
            )
            self.joints[info["name"]] = info
            self.children.setdefault(info["parent"], []).append(info)
        self.lo = np.array([self.joints[n]["lower"] for n in ARM_JOINTS])
        self.hi = np.array([self.joints[n]["upper"] for n in ARM_JOINTS])


    def fk(self, q) -> dict[str, np.ndarray]:
        """q (5,) -> {lien: matrice 4x4 dans world}."""
        values = dict(zip(ARM_JOINTS, q))
        poses = {"base_link": np.eye(4)}
        stack = ["base_link"]
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                T = poses[parent] @ j["origin"]
                if j["type"] == "revolute":
                    T = T @ rot_axis(j["axis"], values.get(j["name"], 0.0))
                poses[j["child"]] = T
                stack.append(j["child"])
        return poses

    def tip(self, q):
        """Position du bout de pince et direction d'approche (unitaire)."""
        P = self.fk(q)
        tip = P[TIP][:3, 3]
        a = tip - P["gripper_link"][:3, 3]
        return tip, a / np.linalg.norm(a)

    # ------------------------------------------------------------------ IK

    def _jac(self, q, eps=1e-6):
        p0, a0 = self.tip(q)
        idx = [ARM_JOINTS.index(n) for n in IK_JOINTS]
        Jp, Ja = np.zeros((3, len(idx))), np.zeros((3, len(idx)))
        for k, i in enumerate(idx):
            dq = q.copy()
            dq[i] += eps
            p, a = self.tip(dq)
            Jp[:, k] = (p - p0) / eps
            Ja[:, k] = (a - a0) / eps
        return p0, a0, Jp, Ja, idx

    def ik(self, target, q0, approach=DOWN, iters=100, damping=1e-3):
        """Renvoie (q, erreur de position en m, écart à la verticale en degrés).
        1) position seule (moindres carrés amortis), 2) verticale dans l'espace nul de la
        position, en refusant tout pas qui dégrade la position."""
        q = np.clip(np.asarray(q0, float).copy(), self.lo, self.hi)
        target = np.asarray(target, float)

        def dpinv(J, d):
            return J.T @ np.linalg.inv(J @ J.T + d**2 * np.eye(J.shape[0]))

        def clip_step(step, m=0.15):
            n = np.linalg.norm(step)
            return step * (m / n) if n > m else step

        for _ in range(iters):
            p, a, Jp, Ja, idx = self._jac(q)
            ep = target - p
            if np.linalg.norm(ep) < 1e-4:
                break
            q[idx] += clip_step(dpinv(Jp, 0.01) @ ep)
            q = np.clip(q, self.lo, self.hi)

        if approach is not None:
            for _ in range(iters):
                p, a, Jp, Ja, idx = self._jac(q)
                ep, ea = target - p, approach - a
                Jp_pinv = dpinv(Jp, damping)
                dq1 = Jp_pinv @ ep
                N = np.eye(len(idx)) - Jp_pinv @ Jp
                dq2 = N @ (dpinv(Ja @ N, 0.05) @ (ea - Ja @ dq1))
                cand = q.copy()
                cand[idx] += clip_step(dq1 + dq2)
                cand = np.clip(cand, self.lo, self.hi)
                pc, ac = self.tip(cand)
                e_old, e_new = np.linalg.norm(ep), np.linalg.norm(target - pc)
                better_dir = ac @ approach > a @ approach + 1e-6
                if e_new <= max(e_old, 5e-4) and (better_dir or e_new < e_old - 1e-6):
                    q = cand
                else:
                    break
        p, a = self.tip(q)
        ang = float(np.degrees(np.arccos(np.clip(a @ (approach if approach is not None else DOWN), -1, 1))))
        return q, float(np.linalg.norm(target - p)), ang

    def ik_best(self, target, q0, tol=2e-3):
        """IK depuis q0, puis depuis quelques postures types si l'erreur reste grande."""
        best = self.ik(target, q0)
        if best[1] < tol and best[2] < 20:
            return best
        pan = float(np.arctan2(target[1], target[0]))
        for lift, elbow, wrist in ((0.0, 0.0, 1.2), (-0.5, 0.8, 1.0), (0.5, -0.5, 1.4), (-1.0, 1.4, 0.4), (0.8, -1.2, 1.5)):
            cand = self.ik(target, np.array([pan, lift, elbow, wrist, q0[4]]))
            if cand[1] < best[1] - 1e-4 or (abs(cand[1] - best[1]) < 5e-4 and cand[2] < best[2]):
                best = cand
            if best[1] < tol and best[2] < 20:
                break
        return best
