import rclpy
import random
from ros2_examples.example_drive_base import DriveExampleNode

from webots_ros2.constants import Power
from std_msgs.msg import Int16

"""
Set Power Example

Used robot model: CompleteRobot
"""

class SetPowerExampleNode(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Power'\n
            """
        )

        super().__init__("set_power_example_node")

        self.srv_left_wheel_vel = self.create_publisher(
            Int16, 
            f"/{self.robot_name}/{self.left_wheel_port}/setPower",
            10
        )
        self.srv_right_wheel_vel = self.create_publisher(
            Int16, 
            f"/{self.robot_name}/{self.right_wheel_port}/setPower",
            10
        )

    def get_random_movement_value(self):
        return 50 # random.randint(1, Power.MAX)

    def send_new_movement(self, left_pow, right_pow):
        self.send_new_pow(left_pow, self.srv_left_wheel_vel)
        self.send_new_pow(right_pow, self.srv_right_wheel_vel)

    def send_new_pow(self, data, pipe):
        req = Int16()
        req.data = data

        pipe.publish(req)


def main(args=None):
    rclpy.init(args=args)

    set_velocity_node = SetPowerExampleNode()

    rclpy.spin(set_velocity_node)

    set_velocity_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()