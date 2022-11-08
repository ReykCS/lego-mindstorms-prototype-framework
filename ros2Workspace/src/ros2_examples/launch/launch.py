from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys
import argparse

from ament_index_python.packages import get_package_share_directory

from lm_ros2_utils.utils import parse_launch_arguments, build_arguments_dict

AVAILABLE_EXAMPLES = [
    "set_velocity", 
    "set_power",
    "set_power_limit",
    "set_velocity_limit",
    "set_position_and_reset_offset",
    "set_position_and_set_offset",
    "set_position",
]

def generate_launch_description(): 
    args = parse_launch_arguments()
    arguments_dict = build_arguments_dict(args.arguments)

    example_name = arguments_dict["example"]
    robot_name = arguments_dict["robot_name"]

    if not example_name in AVAILABLE_EXAMPLES:
        print("Example not available")
        print("Available examples:", AVAILABLE_EXAMPLES)
        sys.exit(1)

    launch_package = get_package_share_directory("lm_ros2_utils")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_package, "launch", "launch_simulation.py")
                ),
                launch_arguments={
                    "robot_name": robot_name
                }.items()
            ),
            Node(
                package="ros2_examples",
                executable=example_name,
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "robot_name": robot_name
                }]
            )
        ]
    )