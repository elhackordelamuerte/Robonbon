#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.
"""
from pathlib import Path

import rclpy
import rclpy.node
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CameraInfo, Image, JointState
from so101_sim import SO101Sim
from std_srvs.srv import Trigger

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Arm joint limits from the URDF (rad). The gripper is commanded in %.
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (0.0, 100.0),
}

# Gripper joint limits in the URDF (rad).
# LeRobot convention: 0 % = closed (lower), 100 % = open (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0
CAMERA_HZ = 10.0
CAMERA_FRAME = "external_cam"
FAKE_PERCEPTION_HZ = 5.0


def _gripper_pct_to_rad(pct: float) -> float:
    lo, hi = GRIPPER_RANGE_RAD
    return lo + min(max(pct, 0.0), 100.0) / 100.0 * (hi - lo)


def _clamp(name: str, value: float) -> float:
    lo, hi = JOINT_LIMITS[name]
    return min(max(value, lo), hi)


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        # Real arm speed limit: max change of the SENT goal per control step (rate limiter on the
        # command). At 50 Hz: 0.05 rad -> 2.5 rad/s, 2 % -> 100 %/s. Not LeRobot max_relative_target:
        # it caps goal - PRESENT, and with the servos' low P gain a loaded joint (shoulder_lift)
        # then stalls under gravity (measured: 27° short of its goal).
        self._max_step_rad: float = self.declare_parameter("max_step_rad", 0.05).value
        self._max_step_gripper: float = self.declare_parameter("max_step_gripper_pct", 2.0).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        # Hold the current pose until the first command, instead of jumping to 0.
        self._obs = self._robot.get_observation()
        self._target = {name: self._obs[f"{name}.pos"] for name in JOINT_NAMES}
        self._sent = dict(self._target)  # rate-limited goal actually sent to the real arm

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)

        if self._use_sim:
            self._bridge = CvBridge()
            self._image_pub = self.create_publisher(Image, "/external_cam/image_raw", 10)
            self._camera_info_pub = self.create_publisher(CameraInfo, "/external_cam/camera_info", 10)
            self._camera_info = self._make_camera_info()
            self.create_timer(1.0 / CAMERA_HZ, self._publish_camera)
            # Ground-truth camera -> world pose (4x4, row-major) for perception_node.
            # Allowed to unblock the pipeline; to be replaced by your own calibration.
            self.declare_parameter("cam_T", self._robot.get_camera_extrinsics().flatten().tolist())

            # Stand-in for perception_node while it doesn't exist: publishes the
            # ground truth from MuJoCo. Off by default, never use it for the evaluation.
            self.declare_parameter("fake_perception", False)
            self._object_pub = self.create_publisher(PoseStamped, "/object_position", 10)
            self._box_pub = self.create_publisher(PoseStamped, "/drop_box_position", 10)
            self.create_timer(1.0 / FAKE_PERCEPTION_HZ, self._publish_ground_truth)
            self.create_service(Trigger, "/reset_object", self._srv_reset_object)

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT) -> SO101Sim | SO101Follower:
        """Creates and connects the backend chosen by use_sim (provided)."""
        if self._use_sim:
            robot = SO101Sim()
            robot.connect()
            return robot

        if not CALIBRATION_FILE.is_file():
            raise RuntimeError(
                f"No calibration file at {CALIBRATION_FILE}: set CALIBRATION_FILE in docker/.env"
            )
        config = SO101FollowerConfig(
            port=port,
            id=CALIBRATION_FILE.stem,
            calibration_dir=CALIBRATION_FILE.parent,
            use_radians=True,
        )
        robot = SO101Follower(config)
        robot.connect(calibrate=False)
        if not robot.is_calibrated:
            robot.disconnect()
            raise RuntimeError(
                f"{CALIBRATION_FILE} does not match the motors: wrong file for this arm? "
                "Re-run lerobot-calibrate on the host."
            )
        # Course repo default: torque disabled, the arm moves freely by hand ("Comment out
        # this line to control the robot."). Commented out here: this driver controls the arm.
        # robot.bus.disable_torque()
        # self.get_logger().info(
        #     "Torque disabled: arm can be moved freely by hand. Remove this part to control the arm."
        # )
        return robot

    def destroy_node(self):
        """Disconnects the backend (provided). On the real arm this disables the torque."""
        if self._robot is not None and self._robot.is_connected:
            self._robot.disconnect()
            self.get_logger().info("Robot disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("/joint_command: name and position sizes differ, ignored")
            return
        for name, value in zip(msg.name, msg.position):
            if name not in self._target:
                self.get_logger().warn(f"/joint_command: unknown joint '{name}'", throttle_duration_sec=2.0)
                continue
            self._target[name] = _clamp(name, value)

    def _rate_limited_goal(self) -> dict:
        """Moves the sent goal towards the target by at most max_step per control step (real arm)."""
        for name, target in self._target.items():
            step = self._max_step_gripper if name == "gripper" else self._max_step_rad
            self._sent[name] += min(max(target - self._sent[name], -step), step)
        return self._sent

    def _control_step(self):
        goal = self._target if self._use_sim else self._rate_limited_goal()
        try:
            self._robot.send_action({f"{name}.pos": value for name, value in goal.items()})
            self._obs = self._robot.get_observation()
        except Exception as e:
            self.get_logger().error(f"Backend error: {e}", throttle_duration_sec=1.0)

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(self._obs[f"{name}.pos"]) for name in JOINT_NAMES[:-1]]
        msg.position.append(_gripper_pct_to_rad(self._obs["gripper.pos"]))
        self._joint_state_pub.publish(msg)

    def _make_camera_info(self) -> CameraInfo:
        K = self._robot.get_camera_intrinsics()
        info = CameraInfo()
        info.header.frame_id = CAMERA_FRAME
        info.width = self._robot.camera_width
        info.height = self._robot.camera_height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = K.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_camera(self):
        img = self._bridge.cv2_to_imgmsg(self._robot.render_camera(), encoding="rgb8")
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = CAMERA_FRAME
        self._camera_info.header.stamp = img.header.stamp
        self._image_pub.publish(img)
        self._camera_info_pub.publish(self._camera_info)

    def _publish_ground_truth(self):
        if not self.get_parameter("fake_perception").value:
            return
        stamp = self.get_clock().now().to_msg()
        obj = PoseStamped()
        obj.header.stamp, obj.header.frame_id = stamp, "world"
        obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = map(float, self._robot.get_object_position())
        w, x, y, z = self._robot.data.body("pickup_object").xquat
        obj.pose.orientation.w, obj.pose.orientation.x = float(w), float(x)
        obj.pose.orientation.y, obj.pose.orientation.z = float(y), float(z)
        self._object_pub.publish(obj)

        box = PoseStamped()
        box.header.stamp, box.header.frame_id = stamp, "world"
        box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, self._robot.get_drop_box_position())
        box.pose.orientation.w = 1.0
        self._box_pub.publish(box)

    def _srv_reset_object(self, request, response):
        self._robot.randomize_object()
        p = self._robot.get_object_position()
        response.success = True
        response.message = f"object at ({p[0]:.3f}, {p[1]:.3f})"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
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
