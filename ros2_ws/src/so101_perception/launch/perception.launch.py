#!/usr/bin/env python3
"""perception.launch.py - perception OpenCV, caméra simulée ou réelle.

    ros2 launch so101_perception perception.launch.py                 # caméra de la sim (/external_cam)
    ros2 launch so101_perception perception.launch.py camera:=real    # caméra réelle (/camera)
    ros2 launch so101_perception perception.launch.py camera:=real source:=0 calibration_file:=/ros2_ws/calib/camera.yaml

camera:=real lance camera_node (flux de l'hôte ou /dev/videoN) ; sans cam_T (calibration de la
pose caméra), perception_node reste en mode 2D : détection + /perception/debug.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    reel = LaunchConfigurationEquals("camera", "real")
    sim = LaunchConfigurationEquals("camera", "sim")
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="sim", choices=["sim", "real"],
                              description="sim: /external_cam (MuJoCo) ; real: camera_node -> /camera"),
        DeclareLaunchArgument("source", default_value="http://host.docker.internal:8081/stream.mjpg",
                              description="Source OpenCV de la caméra réelle (URL ou index)"),
        DeclareLaunchArgument("pose_file", default_value="/ros2_ws/calib/camera_pose.yaml",
                              description="YAML cam_T de la caméra réelle (scripts/calibrate_pose.py, ArUco)"),
        DeclareLaunchArgument("calibration_file", default_value="/ros2_ws/calib/camera.yaml",
                              description="YAML OpenCV (camera_matrix, dist_coeffs) de la caméra réelle"),
        Node(
            package="so101_perception",
            executable="camera",
            name="so101_camera",
            output="screen",
            condition=reel,
            parameters=[{
                "source": LaunchConfiguration("source"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }],
        ),
        # Sim : caméra MuJoCo, pose caméra lue dans le driver, convention d'axes MuJoCo.
        Node(
            package="so101_perception",
            executable="perception",
            name="so101_perception",
            output="screen",
            condition=sim,
            parameters=[{"camera_ns": "/external_cam", "convention_mujoco": True}],
        ),
        # Réel : cam_T à fournir par calibration (solvePnP -> convention OpenCV) ;
        # la boîte marron de la sim n'existe pas : dépôt dans le bol en bois détecté (sinon boite_fixe).
        Node(
            package="so101_perception",
            executable="perception",
            name="so101_perception",
            output="screen",
            condition=reel,
            parameters=[{"camera_ns": "/camera", "convention_mujoco": False, "detecter_boite": False,
                         "cam_T_depuis_driver": False, "aire_min_cube_px": 400, "detecter_bol": True,
                         "cam_T_file": LaunchConfiguration("pose_file")}],
        ),
    ])
