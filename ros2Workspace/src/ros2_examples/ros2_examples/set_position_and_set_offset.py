import math
import rclpy
import time

from rclpy.node import Node
from ros2_examples.example_drive_base import DriveExampleNode

from std_msgs.msg import Float64, Empty

"""
Set Power Example

Used robot model: CompleteRobot
"""

class SetPositionSetOffset(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Positiona and set offset'
            """
        )

        super().__init__("set_position_and_offset", print_movement=False)

        self.pub_left_position = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.left_wheel_port}/setPosition", 
            10
        )
        self.pub_left_position_offset = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.left_wheel_port}/setPositionOffset", 
            10
        )

        self.pub_right_position = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.right_wheel_port}/setPosition", 
            10
        )
        self.pub_right_position_offset = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.right_wheel_port}/setPositionOffset", 
            10
        )

    def set_next_movement(self):
        msg = Float64()
        msg.data = math.pi / 2

        self.pub_left_position.publish(msg)
        self.pub_right_position.publish(msg)

        time.sleep(0.05)
        
        self.pub_left_position_offset.publish(msg)
        self.pub_right_position_offset.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    set_position_offset_node = SetPositionSetOffset()

    rclpy.spin(set_position_offset_node)

    set_position_offset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()