from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

# from webots_ros2.utils import parse_launch_arguments
import os
import sys
# import pathlib

# def get_configuration_file_path(robot_name):
#     package_dir = get_package_share_directory("webots_ros2")
#     filepath = os.path.join(package_dir, "resources", robot_name + ".urdf")

#     return filepath


# def build_arguments_dict(arguments):
#     arg_dict = {}

#     for arg in arguments:
#         argument_name, value = arg.split(":=")

#         if value == None:
#             print("Cannot read value of argument", argument_name, " Please use: <argument_name>:=<value>")
#             sys.exit(1)
    
#         arg_dict[argument_name] = value

#     return arg_dict


def generate_launch_description():

    print("BOOTING")
    # args = parse_launch_arguments()
    # arguments_dict = build_arguments_dict(args.arguments)
    # arg_roboter_name = DeclareLaunchArgument("robot_name")

    # robot_name = arguments_dict["robot_name"]

    # config_file = get_configuration_file_path(robot_name)

    # robot_description = pathlib.Path(config_file).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    launch_description = LaunchDescription([
        # arg_roboter_name, 
        Node(
            package='brickpi3_ros2',
            executable='motor_node',
            output='screen',
            emulate_tty=True, # For showing stdout,
            namespace="CompleteRobot/PORT_A",
            parameters=[
                {
                    'robot_name': "CompleteRobot",
                    'port': "PORT_A",
                    'timestep': 1000,
                    'use_sim_time': use_sim_time
                },
            ],
        )
    ])

    return launch_description