"""Kinematics of the SO-ARM101 from its URDF.

Forward kinematics with ikpy. Inverse kinematics:
  - ik_position: position only (ikpy, several seeds)
  - ik_grasp: position + jaws aligned with a yaw + gripper as vertical as the
    joint limits allow (weighted least squares). ikpy's own orientation mode
    misses the position by 5-8 mm when the wrist is near its limit.
"""
import math
import warnings

import numpy as np
from ikpy.chain import Chain
from scipy.optimize import least_squares

# URDF path from the base to the TCP (gripper_frame_link). In the TCP frame,
# Z is the approach direction and X the jaw closing axis.
CHAIN_ELEMENTS = [
    "base_link", "shoulder_pan", "shoulder_link", "shoulder_lift", "upper_arm_link",
    "elbow_flex", "lower_arm_link", "wrist_flex", "wrist_link", "wrist_roll",
    "gripper_link", "gripper_frame_joint", "gripper_frame_link",
]

# ik_grasp weights: position first, then jaw yaw, then verticality.
W_POS = 1.0
W_JAW = 0.01
W_VERTICAL = 0.003


class Arm:
    def __init__(self, urdf_path: str, seed: int | None = None):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # ikpy warns about the fixed links
            chain = Chain.from_urdf_file(urdf_path, base_elements=CHAIN_ELEMENTS)
            mask = [link.joint_type == "revolute" for link in chain.links]
            self.chain = Chain.from_urdf_file(urdf_path, base_elements=CHAIN_ELEMENTS, active_links_mask=mask)
        self._active = [i for i, active in enumerate(mask) if active]
        self.lower = np.array([self.chain.links[i].bounds[0] for i in self._active])
        self.upper = np.array([self.chain.links[i].bounds[1] for i in self._active])
        self._rng = np.random.default_rng(seed)

    def _full(self, q):
        """5 arm joints -> ikpy vector (one entry per link, fixed ones at 0)."""
        v = np.zeros(len(self.chain.links))
        v[self._active] = q
        return v

    def _seeds(self, seed, n_random):
        if seed is not None and n_random < 0:  # local solve, stays on the same branch
            return [np.clip(seed, self.lower, self.upper)]
        seeds = [np.zeros(5)] if seed is None else [np.clip(seed, self.lower, self.upper), np.zeros(5)]
        return seeds + [self._rng.uniform(self.lower, self.upper) for _ in range(max(n_random, 0))]

    def fk(self, q):
        """4x4 pose of the TCP in base_link."""
        return self.chain.forward_kinematics(self._full(q))

    def tilt(self, q) -> float:
        """Angle between the approach axis and the vertical (deg)."""
        return math.degrees(math.acos(np.clip(-self.fk(q)[2, 2], -1.0, 1.0)))

    def ik_position(self, target, seed=None, n_random=6, tol=0.002):
        """Returns (q, error_m). Several seeds: ikpy alone gets stuck in local minima."""
        target = np.asarray(target, dtype=float)
        best_q, best_err = None, math.inf
        for s in self._seeds(seed, n_random):
            sol = self.chain.inverse_kinematics(target, initial_position=self._full(s))
            err = np.linalg.norm(self.chain.forward_kinematics(sol)[:3, 3] - target)
            if err < best_err:
                best_q, best_err = sol[self._active], err
            if best_err < tol:
                break
        return best_q, best_err

    def ik_grasp(self, target, jaw_yaw, seed=None, n_random=6):
        """Returns (q, error_m, tilt_deg) for a top-down grasp at `target`.
        n_random=-1: only refine from `seed` (for paths, avoids branch jumps)."""
        target = np.asarray(target, dtype=float)
        jaw = np.array([math.cos(jaw_yaw), math.sin(jaw_yaw)])

        def residual(q):
            T = self.fk(q)
            return np.concatenate([
                W_POS * (T[:3, 3] - target),
                W_JAW * (T[:2, 0] - jaw),
                W_VERTICAL * (T[:3, 2] - [0.0, 0.0, -1.0]),
            ])

        best = None
        for s in self._seeds(seed, n_random):
            sol = least_squares(residual, s, bounds=(self.lower, self.upper), xtol=1e-10, ftol=1e-12)
            if best is None or sol.cost < best.cost:
                best = sol
        err = np.linalg.norm(self.fk(best.x)[:3, 3] - target)
        return best.x, err, self.tilt(best.x)
