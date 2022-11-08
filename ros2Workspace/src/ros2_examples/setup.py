from setuptools import setup
import os


PATH = f"{os.getenv('LM_FRAMEWORK_PATH')}ros2Workspace/src/ros2_examples"

package_name = 'ros2_examples'

files_in_dir = (
    lambda dir: 
        [
            f"{dir}/{file}" for file in 
            os.listdir(f"{PATH}/{dir}")
            if os.path.isfile(f"{PATH}/{dir}/{file}")
        ]
    )


data_files = []

data_files.append(("share/" + package_name + "/launch", ["launch/launch.py"]))

data_files.append(('share/' + package_name, ['package.xml']))

script_files = files_in_dir("ros2_examples")
data_files.append(("share/" + package_name + "/ros2_examples", script_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *data_files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reyk',
    maintainer_email='reyk-carstens@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "set_velocity = ros2_examples.set_velocity:main",
            "set_power = ros2_examples.set_power:main",
            "set_power_limit = ros2_examples.set_power_limit:main",
            "set_velocity_limit = ros2_examples.set_velocity_limit:main",
            "set_position_and_reset_offset = ros2_examples.set_position_and_reset_offset:main",
            "set_position = ros2_examples.set_position:main",
            "set_position_and_set_offset = ros2_examples.set_position_and_set_offset:main" 
        ],
    },
)
