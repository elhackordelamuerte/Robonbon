#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics, then pick & place.

Never talks to the backend: reads /joint_states, commands through /joint_command.

  /end_effector_pose    TCP pose from forward kinematics (base_link frame)
  /go_to_target         move the TCP to an XYZ position (inverse kinematics)
  /set_joint_positions  move the 6 joints at once (degrees, gripper 0-100 %)

TODO:
  - Later: pick & place state machine
"""
import math
import warnings
from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from ikpy.chain import Chain
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from so101_interfaces.srv import GoToTarget, SetJointPositions

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]

# URDF path from the base to the TCP (gripper_frame_link, between the jaws).
CHAIN_ELEMENTS = [
    "base_link", "shoulder_pan", "shoulder_link", "shoulder_lift", "upper_arm_link",
    "elbow_flex", "lower_arm_link", "wrist_flex", "wrist_link", "wrist_roll",
    "gripper_link", "gripper_frame_joint", "gripper_frame_link",
]
BASE_FRAME = "base_link"
ACCEPTED_FRAMES = ("", "world", BASE_FRAME)  # world == base_link (static identity TF)

IK_TOLERANCE = 0.002  # m, stop trying other seeds below this
IK_MAX_ERROR = 0.01  # m, reject the target above this
IK_RANDOM_SEEDS = 6
POSE_HZ = 10.0


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")

        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # ikpy warns about the fixed links
            chain = Chain.from_urdf_file(str(urdf), base_elements=CHAIN_ELEMENTS)
            mask = [link.joint_type == "revolute" for link in chain.links]
            self._chain = Chain.from_urdf_file(str(urdf), base_elements=CHAIN_ELEMENTS, active_links_mask=mask)
        self._active = [i for i, active in enumerate(mask) if active]
        self._bounds = [self._chain.links[i].bounds for i in self._active]
        self._rng = np.random.default_rng()

        self._q = None  # arm joints (rad), None until the first /joint_states

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self._pose_pub = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)
        self.create_service(SetJointPositions, "/set_joint_positions", self._srv_set_joint_positions)
        self.create_timer(1.0 / POSE_HZ, self._publish_end_effector_pose)

        self.get_logger().info("Brain node ready.")

    def _cb_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in ARM_JOINT_NAMES):
            self._q = np.array([positions[name] for name in ARM_JOINT_NAMES])

    def _full(self, q):
        """5 arm joints -> ikpy vector (one entry per link, fixed ones at 0)."""
        v = np.zeros(len(self._chain.links))
        v[self._active] = q
        return v

    def _forward_kinematics(self, q):
        """4x4 pose of the TCP in base_link."""
        return self._chain.forward_kinematics(self._full(q))

    def _publish_end_effector_pose(self):
        if self._q is None:
            return
        T = self._forward_kinematics(self._q)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = T[:3, 3]
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self._pose_pub.publish(msg)

    def _solve_ik(self, target_position):
        """Returns (q, error_m). Tries several seeds: ikpy alone gets stuck in local minima."""
        target = np.asarray(target_position, dtype=float)
        seeds = [np.zeros(5)] if self._q is None else [self._q, np.zeros(5)]
        seeds += [np.array([self._rng.uniform(lo, hi) for lo, hi in self._bounds])
                  for _ in range(IK_RANDOM_SEEDS)]

        best_q, best_err = None, math.inf
        for seed in seeds:
            sol = self._chain.inverse_kinematics(target, initial_position=self._full(seed))
            err = np.linalg.norm(self._chain.forward_kinematics(sol)[:3, 3] - target)
            if err < best_err:
                best_q, best_err = sol[self._active], err
            if best_err < IK_TOLERANCE:
                break
        return best_q, best_err

    def _send_joint_command(self, q, gripper_pct=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        if gripper_pct is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_pct))
        self._cmd_pub.publish(msg)

    def _srv_go_to_target(self, request, response):
        frame = request.target_pose.header.frame_id
        if frame not in ACCEPTED_FRAMES:
            response.success = False
            response.message = f"unsupported frame '{frame}', use {BASE_FRAME}"
            return response

        p = request.target_pose.pose.position
        q, err = self._solve_ik([p.x, p.y, p.z])
        if err > IK_MAX_ERROR:
            response.success = False
            response.message = f"unreachable ({err * 1000:.1f} mm away at best)"
            return response

        self._send_joint_command(q)
        response.success = True
        response.message = f"ik error {err * 1000:.2f} mm"
        self.get_logger().info(f"go_to_target ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}): {response.message}")
        return response

    def _srv_set_joint_positions(self, request, response):
        if len(request.positions) != len(JOINT_NAMES):
            response.success = False
            response.message = f"expected {len(JOINT_NAMES)} values ({', '.join(JOINT_NAMES)})"
            return response
        q = np.radians(request.positions[:5])
        self._send_joint_command(q, gripper_pct=request.positions[5])
        response.success = True
        response.message = "sent"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
