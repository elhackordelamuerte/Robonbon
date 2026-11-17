#!/usr/bin/env python3
"""brain_node.py - intelligence of the arm: kinematics, then pick & place.

Never talks to the backend: reads /joint_states, commands through /joint_command.

  /end_effector_pose    TCP pose from forward kinematics (base_link frame)
  /go_to_target         move the TCP to an XYZ position (inverse kinematics)
  /set_joint_positions  move the 6 joints at once (degrees, gripper 0-100 %)
  /pick_and_place       pick the object (/object_position), drop it in the box
                        (/drop_box_position), go back to rest
  /go_to_rest           move the arm to its rest pose
"""
import copy
import json
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
# A pick never starts on an old object position (e.g. perception stopped or lost the cube).
OBJECT_MAX_AGE = 2.0  # s

POSE_HZ = 10.0
TASK_HZ = 50.0

# TCP rest position between two picks. Offset in y so that the arm at rest does not
# hide the drop box from the external camera (the camera -> box ray passes ~(0.20, 0.01, 0.22)).
READY = np.array([0.18, 0.10, 0.17])
GRIPPER_CLOSED = 0.0
PATH_STEP = 0.01  # m between two IK waypoints on straight-line moves

# Durations of the pick & place steps (s).
T_TRAVEL = 3.5
T_APPROACH = 2.0
T_DESCEND = 2.5
T_SEAT = 1.5
T_LIFT = 3.0
T_LOWER = 1.5
T_GRIPPER = 1.0
T_CHECK = 0.8  # arm still after the lift before the grasp check (camera image settles)


def _yaw(q) -> float:
    w, x, y, z = q.w, q.x, q.y, q.z
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _ease(s: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * min(max(s, 0.0), 1.0))


class BrainNode(rclpy.node.Node):
    """Brain of the arm: kinematics, then pick & place."""

    def __init__(self):
        super().__init__("so101_brain")
        # TCP heights (m)
        self.declare_parameter("travel_z", 0.15)  # every horizontal move
        self.declare_parameter("pre_grasp_max_z", 0.10)  # highest point where the gripper is
        self.declare_parameter("pre_grasp_min_z", 0.06)  # searched to be perfectly vertical
        self.declare_parameter("seat_z", 0.032)  # down to here the jaw touches nothing
        self.declare_parameter("grasp_z", 0.018)  # then slowly, until the jaw sits in the hole
        self.declare_parameter("release_z", 0.11)  # above the box
        # The moving jaw goes into the top hole of the cube, the fixed one slides
        # along the outer face. At this opening the moving jaw tip is at
        # (tip_x, tip_y) in the TCP frame (measured on the MuJoCo meshes), and
        # the fixed jaw is ~5 mm away from the face.
        self.declare_parameter("gripper_open", 16.0)
        self.declare_parameter("tip_x", -0.0303)
        self.declare_parameter("tip_y", 0.00015)
        # Slowly bring the arm to its rest pose once /joint_states is received.
        # Set to false on the real arm if it must not move by itself at startup.
        self.declare_parameter("rest_on_start", True)
        self._rest_pending = self.get_parameter("rest_on_start").value
        # Grasp check after the lift (perception: is the cube stuck to the gripper?) and retries:
        # each failed attempt shifts the target by one rectification step (search pattern around
        # the perceived position); the offset that works is kept for the next picks.
        self.declare_parameter("verify_grasp", True)
        self.declare_parameter("max_grasp_attempts", 6)
        self.declare_parameter("rectification_step", 0.005)  # m
        self._attempt = 0
        self._learned_offset = np.zeros(2)  # offset of the last successful grasp (m, base_link)
        self._attempt_offset = np.zeros(2)
        self._check_future = None
        self._verify_client = self.create_client(Trigger, "/perception/verifier_prise")

        urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
        self._arm = Arm(str(urdf))

        self._q = None  # measured arm joints (rad)
        self._gripper = 0.0  # measured gripper (%)
        self._object = None  # last /object_position
        self._object_time = None  # reception time of self._object
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
        self.create_service(Trigger, "/go_to_rest", self._srv_go_to_rest)
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
        if self._rest_pending and self._q is not None and not self._busy:
            self._rest_pending = False
            self._start(self._plan_rest)

    def _cb_object(self, msg: PoseStamped):
        self._object = msg
        self._object_time = self.get_clock().now()

    def _object_age(self) -> float:
        """Seconds since the last /object_position (reception time: stamps may come from another clock)."""
        return (self.get_clock().now() - self._object_time).nanoseconds * 1e-9

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
        elif self._object_age() > OBJECT_MAX_AGE:
            response.success = False
            response.message = (f"/object_position is {self._object_age():.0f} s old: "
                                "object not seen anymore (perception stopped, cube hidden or out of view?)")
        else:
            self._attempt = 0
            obj, box = self._shifted_object(), self._box.pose
            self._start(self._plan_pick_and_place, obj, box)
            response.success = True
            response.message = f"object at ({obj.position.x:.3f}, {obj.position.y:.3f}), planning"
        return response

    # ------------------------------------------------- grasp check & retries

    def _search_offset(self, attempt: int) -> np.ndarray:
        """Search pattern in rectification steps: (0,0), then rings (0,-r), (0,r), (r,0), (-r,0)."""
        if attempt == 0:
            return np.zeros(2)
        ring, k = (attempt - 1) // 4 + 1, (attempt - 1) % 4
        return ring * np.array([(0, -1), (0, 1), (1, 0), (-1, 0)][k], float)

    def _shifted_object(self):
        """Last /object_position + learned offset + search offset of the current attempt."""
        step = self.get_parameter("rectification_step").value
        self._attempt_offset = self._learned_offset + step * self._search_offset(self._attempt)
        obj = copy.deepcopy(self._object.pose)
        obj.position.x += float(self._attempt_offset[0])
        obj.position.y += float(self._attempt_offset[1])
        return obj

    def _grasp_check_result(self) -> bool | None:
        """None while waiting; True/False once the perception answered (True if it cannot tell)."""
        if self._check_future is None:
            if not self._verify_client.service_is_ready():
                self.get_logger().warn("pick & place: /perception/verifier_prise unavailable, grasp not checked")
                return True
            self._check_future = self._verify_client.call_async(Trigger.Request())
            return None
        if not self._check_future.done():
            return None
        result, self._check_future = self._check_future.result(), None
        details = json.loads(result.message) if result.message.startswith("{") else {}
        if details.get("verdict") == "indeterminé":
            self.get_logger().warn(f"pick & place: grasp check inconclusive ({details.get('raison')}), continuing")
            return True
        self.get_logger().info(f"pick & place: grasp {'OK' if result.success else 'FAILED'} "
                               f"(cube {details.get('distance_px', '?')} px from the gripper)")
        return result.success

    def _retry_grasp(self):
        """After a failed grasp (gripper reopened): replan on the fresh cube position, next search offset."""
        self._attempt += 1
        if self._attempt >= self.get_parameter("max_grasp_attempts").value:
            self.get_logger().error(f"pick & place: grasp failed after {self._attempt} attempts, giving up")
            self._steps, self._busy = None, False
            self._start(self._plan_rest)
            return
        if self._object is None or self._object_age() > OBJECT_MAX_AGE:
            self.get_logger().error("pick & place: cube not seen anymore after the failed grasp, giving up")
            self._steps, self._busy = None, False
            self._start(self._plan_rest)
            return
        obj = self._shifted_object()
        self.get_logger().info(f"pick & place: retry {self._attempt + 1}, target offset "
                               f"({self._attempt_offset[0] * 1000:+.0f}, {self._attempt_offset[1] * 1000:+.0f}) mm")
        self._steps = None
        self._start(self._plan_pick_and_place, obj, self._box.pose)

    def _srv_go_to_rest(self, request, response):
        if self._busy or self._q is None:
            response.success, response.message = False, "busy or no /joint_states yet"
        else:
            self._start(self._plan_rest)
            response.success, response.message = True, "going to rest"
        return response

    # ----------------------------------------------------- pick & place

    def _start(self, planner, *args):
        """Planning takes a few seconds of IK: done in a thread, the timer executes."""
        self._busy = True
        q, gripper = self._q.copy(), self._gripper
        threading.Thread(target=self._plan, args=(planner, args, q, gripper), daemon=True).start()

    def _plan(self, planner, args, q_start, gripper_start):
        try:
            steps = planner(*args, q_start)
        except RuntimeError as e:
            self.get_logger().error(f"pick & place: {e}")
            self._busy = False
            return
        self._cmd_q, self._cmd_gripper = q_start, gripper_start
        self._steps = deque(steps)

    def _line(self, p0, p1, yaw0, q0, yaw1=None):
        """Joint waypoints for a straight TCP line p0 -> p1, starting at q0 (included).
        The jaw yaw goes linearly from yaw0 to yaw1."""
        yaw1 = yaw0 if yaw1 is None else yaw1
        p0, p1 = np.asarray(p0), np.asarray(p1)
        steps = max(4, int(np.linalg.norm(p1 - p0) / PATH_STEP))
        qs = [q0]
        for k in range(1, steps + 1):
            s = k / steps
            qs.append(self._arm.ik_grasp(p0 + (p1 - p0) * s, yaw0 + (yaw1 - yaw0) * s, seed=qs[-1], n_random=-1)[0])
        return qs

    def _rest_pose(self, yaw, seed):
        """Rest pose with the jaw yaw (mod 90°) that needs the least wrist roll."""
        options = [self._arm.ik_grasp(READY, yaw + k * math.pi / 2, seed=seed, n_random=2)[0] for k in range(-2, 3)]
        return min(options, key=lambda q: abs(q[4]))

    def _plan_rest(self, q_start):
        q_rest = self._rest_pose(0.0, q_start)
        return [("go to rest", "move", q_rest, GRIPPER_CLOSED, T_TRAVEL)]

    def _plan_pick_and_place(self, obj, box, q_start):
        travel_z = self.get_parameter("travel_z").value
        pre_max = self.get_parameter("pre_grasp_max_z").value
        pre_min = self.get_parameter("pre_grasp_min_z").value
        seat_z = self.get_parameter("seat_z").value
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
            if err < 0.001 and tilt < 0.5:
                candidates.append((abs(q[4]), yaw, xy, q))
        if not candidates:
            raise RuntimeError(f"no vertical grasp at ({center[0]:.3f}, {center[1]:.3f})")
        _, yaw, xy, q_grasp = min(candidates, key=lambda c: c[0])

        # Highest point above the grasp where the gripper is still perfectly
        # vertical: from there the descent is a pure vertical translation.
        z_pre, q_pre = pre_min, None
        for z in np.arange(pre_max, pre_min - 1e-6, -0.005):
            q, err, tilt = self._arm.ik_grasp([xy[0], xy[1], z], yaw, seed=q_grasp, n_random=-1)
            if err < 0.001 and tilt < 0.5:
                z_pre, q_pre = z, q
                break
        if q_pre is None:
            q_pre = self._arm.ik_grasp([xy[0], xy[1], z_pre], yaw, seed=q_grasp)[0]

        def at(z, p=xy):
            return np.array([p[0], p[1], z])

        box_xy = np.array([box.position.x, box.position.y])
        tcp_start = self._arm.fk(q_start)[:3, 3]
        to_cube = self._line(tcp_start, at(travel_z), yaw, q_start)
        approach = self._line(at(travel_z), at(z_pre), yaw, to_cube[-1])
        q_pre = approach[-1]
        descend = self._line(at(z_pre), at(seat_z), yaw, q_pre)
        seat = self._line(at(seat_z), at(grasp_z), yaw, descend[-1])
        lift = self._line(at(grasp_z), at(travel_z), yaw, seat[-1])
        transfer = self._line(at(travel_z), at(travel_z, box_xy), yaw, lift[-1])
        lower = self._line(at(travel_z, box_xy), at(release_z, box_xy), yaw, transfer[-1])
        up = lower[::-1]
        q_rest = self._rest_pose(yaw, up[-1])
        rest_yaw = math.atan2(*self._arm.fk(q_rest)[1::-1, 0])
        rest_yaw = yaw + (rest_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        back = self._line(at(travel_z, box_xy), READY, yaw, up[-1], yaw1=rest_yaw)

        err = np.linalg.norm(self._arm.fk(transfer[-1])[:2, 3] - box_xy)
        if err > IK_MAX_ERROR:
            raise RuntimeError(f"box unreachable ({err * 1000:.1f} mm)")

        self.get_logger().info(
            f"pick & place: planned, jaw yaw {math.degrees(yaw):.0f} deg, vertical from z={z_pre:.3f}")
        return [
            ("move above object", "path", to_cube, GRIPPER_CLOSED, T_TRAVEL),
            ("approach", "path", approach, None, T_APPROACH),
            ("open gripper", "move", q_pre, opened, T_GRIPPER),
            ("align", "wait", None, None, 0.5),
            ("descend", "path", descend, None, T_DESCEND),
            ("seat the jaw", "path", seat, None, T_SEAT),
            ("close gripper", "move", seat[-1], GRIPPER_CLOSED, T_GRIPPER),
            ("hold", "wait", None, None, 0.5),
            ("lift", "path", lift, None, T_LIFT),
            *([("verify grasp", "check", None, None, T_CHECK)] if self.get_parameter("verify_grasp").value else []),
            ("move above box", "path", transfer, None, T_TRAVEL),
            ("lower", "path", lower, None, T_LOWER),
            ("release", "move", lower[-1], opened, T_GRIPPER),
            ("let it fall", "wait", None, None, 1.0),
            ("up", "path", up, None, T_LOWER),
            ("back to rest", "path", back, GRIPPER_CLOSED, T_TRAVEL),
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
        if kind == "check":
            self._send_joint_command(self._cmd_q, self._cmd_gripper)  # hold still
            if now - self._t0 < duration:
                return
            ok = self._grasp_check_result()
            if ok is None:
                return
            self._step = None
            if ok:
                self._learned_offset = self._attempt_offset.copy()
                return
            opened = self.get_parameter("gripper_open").value
            self._steps = deque([("reopen gripper", "move", self._cmd_q, opened, T_GRIPPER),
                                 ("retry", "retry", None, None, 0.1)])
            return
        if kind == "retry":
            self._step = None
            self._retry_grasp()
            return
        s = (now - self._t0) / duration
        e = _ease(s)
        q, g = self._q0, self._g0
        if gripper is not None:
            g = self._g0 + (gripper - self._g0) * e
        if kind == "move":
            q = self._q0 + (np.asarray(target) - self._q0) * e
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
