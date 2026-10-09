#!/usr/bin/env python3
"""brain.launch.py — launches the brain of the arm (IK + pick & place)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rest_on_start", default_value="true",
                              description="Slowly move to the rest pose at startup (false on the real arm)"),
        Node(
            package="so101_brain",
            executable="brain",
            name="so101_brain",
            output="screen",
            parameters=[{
                "rest_on_start": ParameterValue(LaunchConfiguration("rest_on_start"), value_type=bool),
            }],
        ),
    ])
