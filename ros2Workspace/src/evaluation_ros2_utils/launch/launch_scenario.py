from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import Node

import os
import sys

from ament_index_python.packages import get_package_share_directory

from lm_ros2_utils.utils import parse_launch_arguments, build_arguments_dict

AVAILABLE_SCENARIOS = [
    # SIMPLE
    ("set_position_turn_left", "Eval_NoSensors"),
    ("set_position_turn_right", "Eval_NoSensors"),
    
    ("set_velocity_turn_left", "Eval_NoSensors"),
    ("set_velocity_turn_right", "Eval_NoSensors"),

    ("set_power_turn_left", "Eval_NoSensors"),
    ("set_power_turn_right", "Eval_NoSensors"),

    # ADVANCED
    ("light_sensor", "Eval_LightSensor"),
    ("distance_sensor", "Eval_DistanceSensor"),
    ("touch_sensor", "Eval_TouchSensor"),
    ("compass_sensor", "Eval_NoSensors"),
]


def generate_launch_description(): 
    args = parse_launch_arguments()
    arguments_dict = build_arguments_dict(args.arguments)

    arg_evaluation_scenario = DeclareLaunchArgument("evaluation_scenario")

    evaluation_scenario = arguments_dict["evaluation_scenario"]
    robot_name = arguments_dict["robot_name"]
    robot_name = None

    for scenario, r in AVAILABLE_SCENARIOS:
        if scenario == evaluation_scenario:
            robot_name = r

    if not robot_name:
        print("Scenario not available")
        print("Available scenarios:", AVAILABLE_SCENARIOS)
        sys.exit(1)

    lm_ros2_utils = get_package_share_directory("lm_ros2_utils")

    return LaunchDescription(
        [
            arg_evaluation_scenario,
            DeclareLaunchArgument(
                'robot_name',
                default_value = robot_name
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(lm_ros2_utils, "launch", "launch_simulation.py")
                ),
                launch_arguments={
                    "robot_name": robot_name
                }.items()
            ),
            Node(
                package="evaluation_scenarios",
                executable=evaluation_scenario,
                output="screen",
                emulate_tty=True,
            )
        ]
    )