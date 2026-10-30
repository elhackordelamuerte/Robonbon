#!/usr/bin/env python3
"""driver_node.py - ROS2 driver of the SO-ARM101.

The only node that talks to the backend: SO101Sim (MuJoCo) or SO101Follower
(real arm), chosen by the `use_sim` parameter. The rest of the stack never
knows which one runs.

TODO:
  - Publish /joint_states in RADIANS (>= 20 Hz)
  - Subscribe to /joint_command (radians, gripper 0-100 %)
  - Disconnect the backend when the node stops
"""
from pathlib import Path

import rclpy
import rclpy.node
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from so101_sim import SO101Sim

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


def _gripper_pct_to_rad(pct: float) -> float:
    return GRIPPER_RANGE_RAD[0] + (pct / 100.0) * (GRIPPER_RANGE_RAD[1] - GRIPPER_RANGE_RAD[0])


class DriverNode(rclpy.node.Node):
    """Driver SO-ARM101: /joint_command -> backend -> /joint_states."""

    def __init__(self):
        super().__init__("so101_driver")
        self._use_sim: bool = self.declare_parameter("use_sim", True).value
        port: str = self.declare_parameter("port", DEFAULT_PORT).value
        mode = "simulation" if self._use_sim else "hardware"
        self.get_logger().info(f"Mode: {mode} (port={port})")

        self._robot = self._connect_arm(port)
        self.get_logger().info(f"Robot connected: {type(self._robot).__name__}")

        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 20)
        self._joint_timer = self.create_timer(1.0 / JOINT_STATE_HZ, self._publish_joint_states)

        # Latest target per joint (radians, gripper 0-100 %), sent by _control_step.
        self._target: dict[str, float] = {}
        self._has_new_target = False
        self._joint_command_sub = self.create_subscription(JointState, "/joint_command", self._cb_joint_command, 10)
        self._control_timer = self.create_timer(1.0 / CONTROL_HZ, self._control_step)

        # TODO: create the publishers, subscribers and timers.

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
        return robot

    def destroy_node(self):
        # TODO: disconnect the backend
        super().destroy_node()
        self._robot.disconnect()

    def _cb_joint_command(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("/joint_command: name and position lengths differ, ignored")
            return
        for name, position in zip(msg.name, msg.position):
            if name not in JOINT_NAMES:
                self.get_logger().warn(f"/joint_command: unknown joint '{name}', ignored")
                continue
            # Partial commands only update the given joints, the others keep their target.
            self._target[name] = float(position)
            self._has_new_target = True

    def _control_step(self):
        """Forwards the latest /joint_command target to the backend."""
        if not self._has_new_target:
            return
        action = {f"{name}.pos": value for name, value in self._target.items()}
        try:
            self._robot.send_action(action)
        except Exception as e:
            self.get_logger().error(f"send_action failed: {e}", throttle_duration_sec=1.0)
            return
        self._has_new_target = False

    def _publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = JOINT_NAMES

        observation = self._robot.get_observation()
        joint_state_msg.position = []

        for joint in JOINT_NAMES:
            if joint == "gripper":
                joint_state_msg.position.append(_gripper_pct_to_rad(observation[joint+".pos"]))
            else:
                joint_state_msg.position.append(observation[joint+".pos"])

        self._joint_state_pub.publish(joint_state_msg)


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
