from contextlib import closing
import rclpy
import random
from ros2_examples.example_drive_base import DriveExampleNode

from std_msgs.msg import Int16, Int16MultiArray

from brickpi3_ros2.constants import Power

"""
Set Power Limit Example

Used robot model: CompleteRobot
"""

class SetPowerLimitExampleNode(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Power Limit'\n
            """
        )

        super().__init__("set_power_limit_example_node")

        self.pub_left_wheel_pow = self.create_publisher(
            Int16, 
            f"/CompleteRobot/{self.left_wheel_port}/setPower",
            10
        )
        self.pub_right_wheel_pow = self.create_publisher(
            Int16, 
            f"/CompleteRobot/{self.right_wheel_port}/setPower",
            10
        )

        self.pub_left_wheel_pow_limit = self.create_publisher(
            Int16MultiArray, 
            f"/CompleteRobot/{self.left_wheel_port}/setPowerLimit",
            10
        )
        self.pub_right_wheel_pow_limit = self.create_publisher(
            Int16MultiArray, 
            f"/CompleteRobot/{self.right_wheel_port}/setPowerLimit",
            10
        )

        self.max_power = 50
        self.increasing = False

    def set_next_movement(self):
        super().set_next_movement()

        self.max_power = self.max_power - (5 if not self.increasing else -5)

        if self.max_power <= 5 or self.max_power >= 95:
            self.increasing = not self.increasing

        self.send_new_power_limits()

    def get_random_movement_value(self):
        return self.max_power

    def send_new_movement(self, left_pow, right_pow):
        self.send_new_pow(left_pow, self.pub_left_wheel_pow)
        self.send_new_pow(right_pow, self.pub_right_wheel_pow)

    def send_new_power_limits(self):
        power_limits = [-self.max_power, self.max_power]

        self.send_new_pow_limit(power_limits, self.pub_left_wheel_pow_limit)
        self.send_new_pow_limit(power_limits, self.pub_right_wheel_pow_limit)

    def send_new_pow(self, data, pipe):
        req = Int16()
        req.data = data

        pipe.publish(req)

    def send_new_pow_limit(self, data, pipe):
        req = Int16MultiArray()
        req.data = data

        pipe.publish(req)
   

def main(args=None):
    rclpy.init(args=args)

    set_velocity_node = SetPowerLimitExampleNode()

    rclpy.spin(set_velocity_node)

    set_velocity_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()