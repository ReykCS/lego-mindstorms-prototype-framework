from setuptools import setup
import os


PATH = f"{os.getenv('LM_FRAMEWORK_PATH')}ros2Workspace/src/webots_ros2"

files_in_dir = (
    lambda dir: 
        [
            f"{dir}/{file}" for file in 
            os.listdir(f"{PATH}/{dir}")
            if os.path.isfile(f"{PATH}/{dir}/{file}")
        ]
    )


package_name = "webots_ros2"
data_files = []

script_files = files_in_dir("webots_ros2")
data_files.append(("share/" + package_name + "/webots_ros2", script_files))


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        *data_files
    ],
    install_requires=["setuptools", "launch"],    
    entry_points={
        "console_scripts": [
            # "motor_node = webots_ros2.motor_node:main",
            # "distance_sensor_node = webots_ros2.distance_sensor_node:main",
            # "clock_node = webots_ros2.clock_node:main"
            "webots_node = webots_ros2.webots_node:main"
        ]
    }
)