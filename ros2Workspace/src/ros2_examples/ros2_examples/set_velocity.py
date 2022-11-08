import rclpy
import random
from ros2_examples.example_drive_base import DriveExampleNode

from std_msgs.msg import Float32

from webots_ros2.constants import Velocity

"""
Set Velocity Example

Used robot model: CompleteRobot

Lets the robot drive forward and backwards randomly
"""

class SetVelocityExampleNode(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Velocity'\n
            """
        )

        super().__init__("set_velocity_example_node")

        self.pub_left_wheel_vel = self.create_publisher(
            Float32, 
            f"/{self.robot_name}/{self.left_wheel_port}/setVelocity",
            10
        )
        self.pub_right_wheel_vel = self.create_publisher(
            Float32, 
            f"/{self.robot_name}/{self.right_wheel_port}/setVelocity",
            10
        )

    def get_random_movement_value(self):
        return random.uniform(0.0001, Velocity.MAX)

    def send_new_movement(self, left_vel, right_vel):
        self.send_new_vel(left_vel, self.pub_left_wheel_vel)
        self.send_new_vel(right_vel, self.pub_right_wheel_vel)

    def send_new_vel(self, vel, pipe):
        msg = Float32()
        msg.data = float(vel)

        pipe.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    set_velocity_node = SetVelocityExampleNode()

    rclpy.spin(set_velocity_node)

    set_velocity_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()