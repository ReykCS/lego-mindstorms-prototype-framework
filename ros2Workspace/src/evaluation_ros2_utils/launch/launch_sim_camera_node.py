import os
import sys
import pathlib

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    os.environ["WEBOTS_ROBOT_NAME"] = "camera"

    package_dir = get_package_share_directory("evaluation_ros2_utils")
    filepath = os.path.join(package_dir, "resource", "Camera.urdf")
    robot_description = pathlib.Path(filepath).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    launch_description = LaunchDescription([
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            emulate_tty=True, # For showing stdout,
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True
                },
            ],
        )
    ])

    return launch_description