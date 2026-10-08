#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics, then pick & place.

Never talks to the backend: reads /joint_states, commands through /joint_command.

  /end_effector_pose    TCP pose from forward kinematics (base_link frame)
  /go_to_target         move the TCP to an XYZ position (inverse kinematics)
  /set_joint_positions  move the 6 joints at once (degrees, gripper 0-100 %)
  /pick_and_place       pick the object (/object_position), drop it in the box
                        (/drop_box_position), go back home
"""
import math
import threading
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from so101_interfaces.srv import GoToTarget, SetJointPositions
from std_srvs.srv import Trigger

from so101_brain.kinematics import Arm

ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
JOINT_NAMES = ARM_JOINT_NAMES + ["gripper"]
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)  # 0 % .. 100 %

BASE_FRAME = "base_link"
ACCEPTED_FRAMES = ("", "world", BASE_FRAME)  # world == base_link (static identity TF)
IK_MAX_ERROR = 0.01  # m

POSE_HZ = 10.0
TASK_HZ = 50.0

HOME = np.array([0.0, -1.2, 1.2, 0.9, 0.0])
GRIPPER_CLOSED = 0.0
PATH_STEPS = 8  # IK waypoints on straight-line moves

# Durations of the pick & place steps (s).
T_MOVE = 3.5
T_DESCEND = 3.0
T_LIFT = 2.5
T_GRIPPER = 1.0


def _yaw(q) -> float:
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _ease(s: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * min(max(s, 0.0), 1.0))


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        self.declare_parameter("hover_z", 0.10)  # TCP height above the object before descending
        self.declare_parameter("grasp_z", 0.015)  # TCP height when closing, jaw deep in the hole
        self.declare_parameter("release_z", 0.13)  # TCP height above the box when opening
        # The moving jaw goes into the top hole of the cube, the fixed one slides
        # along the outer face. At this opening the moving jaw tip is at
        # (tip_x, tip_y) in the TCP frame (measured on the MuJoCo meshes), and
        # the fixed jaw is ~5 mm away from the face.
        self.declare_parameter("gripper_open", 16.0)
        self.declare_parameter("tip_x", -0.0303)
        self.declare_parameter("tip_y", 0.00015)

        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._arm = Arm(str(urdf))

        self._q = None  # measured arm joints (rad)
        self._gripper = 0.0  # measured gripper (%)
        self._object = None  # last /object_position
        self._box = None  # last /drop_box_position

        self._steps = None  # pick & place steps being executed
        self._step = None
        self._busy = False

        self.create_subscription(JointState, "/joint_states", self._cb_joint_states, 10)
        self.create_subscription(PoseStamped, "/object_position", self._cb_object, 10)
        self.create_subscription(PoseStamped, "/drop_box_position", self._cb_box, 10)
        self._pose_pub = self.create_publisher(PoseStamped, "/end_effector_pose", 10)
        self._cmd_pub = self.create_publisher(JointState, "/joint_command", 10)
        self.create_service(GoToTarget, "/go_to_target", self._srv_go_to_target)
        self.create_service(SetJointPositions, "/set_joint_positions", self._srv_set_joint_positions)
        self.create_service(Trigger, "/pick_and_place", self._srv_pick_and_place)
        self.create_timer(1.0 / POSE_HZ, self._publish_end_effector_pose)
        self.create_timer(1.0 / TASK_HZ, self._run_task)

        self.get_logger().info("Brain node ready.")

    # ------------------------------------------------------------ inputs

    def _cb_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in ARM_JOINT_NAMES):
            self._q = np.array([positions[name] for name in ARM_JOINT_NAMES])
        if "gripper" in positions:
            lo, hi = GRIPPER_RANGE_RAD
            self._gripper = (positions["gripper"] - lo) / (hi - lo) * 100.0

    def _cb_object(self, msg: PoseStamped):
        self._object = msg

    def _cb_box(self, msg: PoseStamped):
        self._box = msg

    # -------------------------------------------------------- kinematics

    def _forward_kinematics(self, q):
        return self._arm.fk(q)

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
        return self._arm.ik_position(target_position, seed=self._q)

    def _send_joint_command(self, q, gripper_pct=None):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        if gripper_pct is not None:
            msg.name.append("gripper")
            msg.position.append(float(gripper_pct))
        self._cmd_pub.publish(msg)

    # ---------------------------------------------------------- services

    def _srv_go_to_target(self, request, response):
        frame = request.target_pose.header.frame_id
        if self._busy:
            response.success, response.message = False, "busy (pick & place running)"
            return response
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
        if self._busy:
            response.success, response.message = False, "busy (pick & place running)"
            return response
        if len(request.positions) != len(JOINT_NAMES):
            response.success = False
            response.message = f"expected {len(JOINT_NAMES)} values ({', '.join(JOINT_NAMES)})"
            return response
        q = np.radians(request.positions[:5])
        self._send_joint_command(q, gripper_pct=request.positions[5])
        response.success = True
        response.message = "sent"
        return response

    def _srv_pick_and_place(self, request, response):
        if self._busy:
            response.success, response.message = False, "already running"
        elif self._q is None or self._object is None or self._box is None:
            response.success = False
            response.message = "waiting for /joint_states, /object_position and /drop_box_position"
        else:
            # Planning takes a few seconds of IK: done in a thread, the timer executes.
            self._busy = True
            obj, box, q = self._object.pose, self._box.pose, self._q.copy()
            threading.Thread(target=self._plan, args=(obj, box, q, self._gripper), daemon=True).start()
            response.success = True
            response.message = f"object at ({obj.position.x:.3f}, {obj.position.y:.3f}), planning"
        return response

    # ----------------------------------------------------- pick & place

    def _plan(self, obj, box, q_start, gripper_start):
        try:
            steps = self._plan_pick_and_place(obj, box, q_start)
        except RuntimeError as e:
            self.get_logger().error(f"pick & place: {e}")
            self._busy = False
            return
        self._cmd_q, self._cmd_gripper = q_start, gripper_start
        self._steps = deque(steps)

    def _plan_pick_and_place(self, obj, box, q_start):
        hover_z = self.get_parameter("hover_z").value
        grasp_z = self.get_parameter("grasp_z").value
        release_z = self.get_parameter("release_z").value
        opened = self.get_parameter("gripper_open").value
        tip_x = self.get_parameter("tip_x").value
        tip_y = self.get_parameter("tip_y").value

        # The cube is invariant under 90° rotations: try the 4 sides for the fixed
        # jaw, keep the one that needs the least wrist roll.
        center = np.array([obj.position.x, obj.position.y])
        cube_yaw = _yaw(obj.orientation) % (math.pi / 2)
        candidates = []
        for k in range(4):
            yaw = cube_yaw + k * math.pi / 2
            jaw_axis = np.array([math.cos(yaw), math.sin(yaw)])  # TCP X, top-down
            side_axis = np.array([math.sin(yaw), -math.cos(yaw)])  # TCP Y
            xy = center - tip_x * jaw_axis - tip_y * side_axis  # moving jaw tip over the hole
            grasp = np.array([xy[0], xy[1], grasp_z])
            q, err, tilt = self._arm.ik_grasp(grasp, yaw, seed=q_start)
            if err < 0.002 and tilt < 5.0:
                candidates.append((abs(q[4]), yaw, grasp, q))
        if not candidates:
            raise RuntimeError(f"no top-down grasp at ({center[0]:.3f}, {center[1]:.3f})")
        _, yaw, grasp, q_grasp = min(candidates, key=lambda c: c[0])
        above = np.array([grasp[0], grasp[1], hover_z])
        q_above, err, tilt = self._arm.ik_grasp(above, yaw, seed=q_grasp)
        if err > 0.002:
            raise RuntimeError(f"cannot hover above the object ({err * 1000:.1f} mm)")

        down = [q_above]
        for k in range(1, PATH_STEPS + 1):
            p = above + (grasp - above) * k / PATH_STEPS
            down.append(self._arm.ik_grasp(p, yaw, seed=down[-1], n_random=-1)[0])
        up = down[::-1]

        drop = np.array([box.position.x, box.position.y, release_z])
        q_drop, err = self._arm.ik_position(drop, seed=q_above)
        if err > IK_MAX_ERROR:
            raise RuntimeError(f"box unreachable ({err * 1000:.1f} mm)")

        self.get_logger().info(f"pick & place: planned, jaw yaw {math.degrees(yaw):.0f} deg, hover tilt {tilt:.0f} deg")
        return [
            ("move above object", "move", q_above, GRIPPER_CLOSED, T_MOVE),
            ("open gripper", "move", q_above, opened, T_GRIPPER),
            ("align", "wait", None, None, 0.5),
            ("descend", "path", down, None, T_DESCEND),
            ("close gripper", "move", down[-1], GRIPPER_CLOSED, T_GRIPPER),
            ("hold", "wait", None, None, 0.5),
            ("lift", "path", up, None, T_LIFT),
            ("move above box", "move", q_drop, None, T_MOVE),
            ("release", "move", q_drop, opened, T_GRIPPER),
            ("let it fall", "wait", None, None, 1.0),
            ("go home", "move", HOME, GRIPPER_CLOSED, T_MOVE),
        ]

    def _run_task(self):
        if self._steps is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._step is None:
            if not self._steps:
                self._steps, self._busy = None, False
                self.get_logger().info("pick & place: done")
                return
            self._step = self._steps.popleft()
            self._t0, self._q0, self._g0 = now, self._cmd_q, self._cmd_gripper
            self.get_logger().info(f"pick & place: {self._step[0]}")

        name, kind, target, gripper, duration = self._step
        s = (now - self._t0) / duration
        e = _ease(s)
        q, g = self._q0, self._g0
        if kind == "move":
            q = self._q0 + (np.asarray(target) - self._q0) * e
            if gripper is not None:
                g = self._g0 + (gripper - self._g0) * e
        elif kind == "path":
            x = e * (len(target) - 1)
            k = min(int(x), len(target) - 2)
            q = target[k] + (target[k + 1] - target[k]) * (x - k)
        self._send_joint_command(q, g)

        if s >= 1.0:
            self._cmd_q, self._cmd_gripper = q, g
            self._step = None


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
