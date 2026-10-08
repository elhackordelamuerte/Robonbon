#!/usr/bin/env python3
"""ik_accuracy.py - measures how close the TCP gets to random XYZ targets.

    ros2 run so101_brain ik_accuracy            # 20 targets
    ros2 run so101_brain ik_accuracy 50

Error = distance between the target and /end_effector_pose once the arm has
settled, i.e. FK of the measured joints (matches MuJoCo to < 0.01 mm).
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from so101_interfaces.srv import GoToTarget

# Workspace in front of the robot, above the table (m, base_link).
X_RANGE = (0.15, 0.32)
Y_RANGE = (-0.20, 0.20)
Z_RANGE = (0.05, 0.25)
SETTLE_TOL = 0.0002  # m of motion between two poses = settled
SETTLE_TIMEOUT = 6.0


class IkAccuracy(Node):
    def __init__(self):
        super().__init__("ik_accuracy")
        self.pose = None
        self.create_subscription(PoseStamped, "/end_effector_pose", self._cb_pose, 10)
        self.client = self.create_client(GoToTarget, "/go_to_target")

    def _cb_pose(self, msg):
        p = msg.pose.position
        self.pose = np.array([p.x, p.y, p.z])

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go_to(self, target):
        req = GoToTarget.Request()
        req.target_pose.header.frame_id = "base_link"
        req.target_pose.pose.position.x, req.target_pose.pose.position.y, req.target_pose.pose.position.z = target
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def settled_pose(self):
        self.wait(0.5)
        start = time.monotonic()
        last = self.pose
        while time.monotonic() - start < SETTLE_TIMEOUT:
            self.wait(0.3)
            if np.linalg.norm(self.pose - last) < SETTLE_TOL:
                break
            last = self.pose
        return self.pose


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    rclpy.init()
    node = IkAccuracy()
    node.client.wait_for_service()
    while node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    rng = np.random.default_rng(0)
    errors, unreachable = [], 0
    for i in range(n):
        target = np.array([rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE), rng.uniform(*Z_RANGE)])
        res = node.go_to(target)
        if not res.success:
            unreachable += 1
            print(f"{i + 1:3d}  target {np.round(target, 3)}  {res.message}")
            continue
        err = np.linalg.norm(node.settled_pose() - target)
        errors.append(err)
        print(f"{i + 1:3d}  target {np.round(target, 3)}  error {err * 1000:6.2f} mm")

    errors = np.array(errors) * 1000
    print(f"\nreached {len(errors)}/{n}, unreachable {unreachable}")
    if len(errors):
        print(f"error mean {errors.mean():.2f} mm  median {np.median(errors):.2f} mm  "
              f"max {errors.max():.2f} mm  under 10 mm: {(errors < 10).sum()}/{len(errors)}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
