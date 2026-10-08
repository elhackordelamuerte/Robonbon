#!/usr/bin/env python3
"""driver.launch.py - launches the SO-ARM101 driver node

    ros2 launch so101_driver driver.launch.py                # simulation
    ros2 launch so101_driver driver.launch.py use_sim:=false # real robot
    ros2 launch so101_driver driver.launch.py fake_joints:=true  # links visibles dans RViz (pose zéro)

fake_joints:=true lance joint_state_publisher : publie /joint_states à zéro pour que
robot_state_publisher crée les TF de tous les links (bras complet dans RViz).
À désactiver dès que driver_node publie /joint_states (sinon deux sources en conflit).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="Simulated backend or real robot"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",  #  Use the following command to find the robot port:
                              # sudo udevadm monitor --subsystem-match=tty --property | grep --line-buffered -E "(DEVNAME=|ACTION=)"
                              description="Serial port of the real SO-ARM101"),
        DeclareLaunchArgument("fake_joints", default_value="false",
                              description="Publish fake /joint_states (zero pose) to display the links in RViz"),
        Node(
            package="so101_driver",
            executable="driver",
            name="so101_driver",
            output="screen",
            parameters=[{
                "use_sim": ParameterValue(LaunchConfiguration("use_sim"), value_type=bool),
                "port": LaunchConfiguration("port"),
            }],
        ),
        # Pose zéro factice : uniquement tant que driver_node ne publie pas /joint_states.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("fake_joints")),
        ),
    ])
