from contextlib import closing
import rclpy
import random
from ros2_examples.example_drive_base import DriveExampleNode

from std_msgs.msg import Float32, Float32MultiArray

from brickpi3_ros2.constants import Power, Velocity

"""
Set Velocity Limit Example

Used robot model: CompleteRobot
"""

class SetVelocityLimitExampleNode(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Velocity Limit'\n
            """
        )

        super().__init__("set_velocity_limit_example_node")

        self.pub_left_wheel_vel = self.create_publisher(
            Float32, 
            f"/CompleteRobot/{self.left_wheel_port}/setVelocity",
            10
        )
        self.pub_right_wheel_vel = self.create_publisher(
            Float32, 
            f"/CompleteRobot/{self.right_wheel_port}/setVelocity",
            10
        )

        self.pub_left_wheel_vel_limit = self.create_publisher(
            Float32MultiArray, 
            f"/CompleteRobot/{self.left_wheel_port}/setVelocityLimit",
            10
        )
        self.pub_right_wheel_vel_limit = self.create_publisher(
            Float32MultiArray, 
            f"/CompleteRobot/{self.right_wheel_port}/setVelocityLimit",
            10
        )

        self.max_velocity = Velocity.MAX / 2
        self.increasing = False

    def set_next_movement(self):
        super().set_next_movement()

        self.max_velocity = self.max_velocity - (0.05 if not self.increasing else -0.05)

        if self.max_velocity <= Velocity.MAX / 10 or self.max_velocity >= 9 * Velocity.MAX / 10:
            self.increasing = not self.increasing

        self.send_new_velocity_limits()

    def get_random_movement_value(self):
        return self.max_velocity

    def send_new_movement(self, left_vel, right_vel):
        self.send_new_vel(left_vel, self.pub_left_wheel_vel)
        self.send_new_vel(right_vel, self.pub_right_wheel_vel)

    def send_new_velocity_limits(self):
        velocity_limits = [-self.max_velocity, self.max_velocity]

        self.send_new_vel_limit(velocity_limits, self.pub_left_wheel_vel_limit)
        self.send_new_vel_limit(velocity_limits, self.pub_right_wheel_vel_limit)

    def send_new_vel(self, data, pipe):
        req = Float32()
        req.data = float(data)

        pipe.publish(req)

    def send_new_vel_limit(self, data, pipe):
        req = Float32MultiArray()
        req.data = data

        pipe.publish(req)
   

def main(args=None):
    rclpy.init(args=args)

    set_velocity_node = SetVelocityLimitExampleNode()

    rclpy.spin(set_velocity_node)

    set_velocity_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()