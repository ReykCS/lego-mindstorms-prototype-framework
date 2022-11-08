from setuptools import setup


package_name = 'evaluation_ros2_utils'
data_files = []

data_files.append(("share/" + package_name + "/" + package_name, [package_name + "/camera_plugin.py"]))
data_files.append(("share/" + package_name + "/resource", ["resource/Camera.urdf"]))
data_files.append(("share/" + package_name + "/launch", ["launch/launch_scenario.py", "launch/launch_sim_camera_node.py"]))


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
        ],
    },
)
