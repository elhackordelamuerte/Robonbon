#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

  - /joint_command (sub) : JointState, radians, gripper 0-100 %
  - /joint_states  (pub) : JointState, radians (gripper inclus), 30 Hz
  - /external_cam/image_raw (pub) : Image RGB, ~10 Hz (simulation uniquement)

Test manuel (éviter --once : le message unique part souvent avant la découverte DDS) :
    ros2 topic pub -r 10 --times 10 /joint_command sensor_msgs/msg/JointState \
      "{name: [shoulder_pan, elbow_flex, gripper], position: [0.5, -0.8, 80.0]}"
"""
from pathlib import Path

import rclpy
import rclpy.node
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, JointState

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Gripper joint limits in the URDF (rad).
# LeRobot convention: 0 % = closed (lower), 100 % = open (upper).
GRIPPER_RANGE_RAD = (-0.174533, 1.74533)

# Calibration file of the real arm, mounted by docker compose (CALIBRATION_FILE in docker/.env).
CALIBRATION_FILE = Path("/calibration/arm.json")
DEFAULT_PORT = "/dev/ttyACM0"

CONTROL_HZ = 50.0
JOINT_STATE_HZ = 30.0
CAMERA_HZ = 10.0
CAMERA_FRAME_ID = "external_cam"


def _gripper_pct_to_rad(pct: float) -> float:
    """Ouverture pince 0-100 % -> angle URDF (rad), bornée."""
    lo, hi = GRIPPER_RANGE_RAD
    pct = min(max(pct, 0.0), 100.0)
    return lo + pct / 100.0 * (hi - lo)


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self._observation: dict = self._robot.get_observation()
        # Consigne courante (clés '<joint>.pos') : vide tant qu'aucune commande reçue,
        # le bras garde alors sa position.
        self._target: dict = {}
        self._bridge = CvBridge()

        self._pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_image = (
            self.create_publisher(Image, "/external_cam/image_raw", 1) if self._use_sim else None
        )
        self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)

        self.create_timer(1.0 / CONTROL_HZ, self._control_step)
        self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)
        if self._use_sim:
            self.create_timer(1.0 / CAMERA_HZ, self._publish_camera)

        self.get_logger().info("Driver node ready.")

    def _connect_arm(self, port: str = DEFAULT_PORT):
        """Instancie et connecte le backend choisi par use_sim."""
        if self._use_sim:
            from so101_sim import SO101Sim

            robot = SO101Sim()
        else:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

            if not CALIBRATION_FILE.is_file() or CALIBRATION_FILE.stat().st_size == 0:
                raise RuntimeError(
                    f"Calibration introuvable ({CALIBRATION_FILE}) : renseigner CALIBRATION_FILE dans docker/.env"
                )
            config = SO101FollowerConfig(
                port=port,
                id=CALIBRATION_FILE.stem,
                calibration_dir=CALIBRATION_FILE.parent,
                use_radians=True,
            )
            robot = SO101Follower(config)
        robot.connect(calibrate=False)
        return robot

    def destroy_node(self):
        robot = getattr(self, "_robot", None)
        if robot is not None and robot.is_connected:
            robot.disconnect()
            self.get_logger().info("Backend disconnected.")
        super().destroy_node()

    def _cb_joint_command(self, msg: JointState):
        """Met à jour la consigne ; les joints absents du message gardent leur cible."""
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("/joint_command ignoré : name et position de tailles différentes")
            return
        update = {
            f"{name}.pos": float(pos)
            for name, pos in zip(msg.name, msg.position, strict=True)
            if name in JOINT_NAMES
        }
        unknown = set(msg.name) - set(JOINT_NAMES)
        if unknown:
            self.get_logger().warn(f"Joints inconnus ignorés : {sorted(unknown)}")
        self._target = {**self._target, **update}

    def _control_step(self):
        """Envoie la consigne au backend puis lit l'état (fait aussi avancer la sim)."""
        if self._target:
            self._robot.send_action(self._target)
        self._observation = self._robot.get_observation()

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [
            _gripper_pct_to_rad(self._observation["gripper.pos"])
            if name == "gripper"
            else self._observation[f"{name}.pos"]
            for name in JOINT_NAMES
        ]
        self._pub_joint_states.publish(msg)

    def _publish_camera(self):
        frame = self._robot.render_camera()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="rgb8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_FRAME_ID
        self._pub_image.publish(msg)


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
