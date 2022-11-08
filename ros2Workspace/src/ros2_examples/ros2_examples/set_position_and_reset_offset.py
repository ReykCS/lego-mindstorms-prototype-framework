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

class SetPositionResetOffset(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Positiona and reset offset'
            """
        )

        super().__init__("set_position_and_reset_offset", print_movement=False)

        self.pub_left_position = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.left_wheel_port}/setPosition", 
            10
        )
        self.pub_left_position_reset_offset = self.create_publisher(
            Empty, 
            f"/CompleteRobot/{self.left_wheel_port}/resetPositionOffset", 
            10
        )

        self.pub_right_position = self.create_publisher(
            Float64, 
            f"/CompleteRobot/{self.right_wheel_port}/setPosition", 
            10
        )
        self.pub_right_position_reset_offset = self.create_publisher(
            Empty, 
            f"/CompleteRobot/{self.right_wheel_port}/resetPositionOffset", 
            10
        )

    def set_next_movement(self):
        self.pub_left_position_reset_offset.publish(Empty())
        self.pub_right_position_reset_offset.publish(Empty())

        # Necessary so that offset is processed before position does
        time.sleep(0.05)

        msg = Float64()
        msg.data = math.pi / 2

        self.pub_left_position.publish(msg)
        self.pub_right_position.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    set_position_reset_offset_node = SetPositionResetOffset()

    rclpy.spin(set_position_reset_offset_node)

    set_position_reset_offset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()