"""Shows the SAC model in RViz without a simulation, with sliders for the joints.

Useful to check frames and sensor mounts after changing urdf/sac.urdf.xacro.
Needs rviz2 and joint_state_publisher_gui (not in the headless simulation image).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_directory = get_package_share_directory("gazebo_environment")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(package_directory, "urdf", "sac.urdf.xacro"), " namespace:=sac"]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(package_directory, "rviz", "config.rviz")],
            ),
        ]
    )
