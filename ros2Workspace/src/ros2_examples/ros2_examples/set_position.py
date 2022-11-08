import math
import rclpy

from ros2_examples.example_drive_base import DriveExampleNode

from std_msgs.msg import Float64

"""
Set Power Example

Used robot model: EvalWheeledRoboterNoSensors
"""

class SetPosition(DriveExampleNode):
    def __init__(self):
        print(
            """
            Starting Example 'Set Position'
            """
        )

        super().__init__("set_position", print_movement=False)

        self.pub_position_left = self.create_publisher(
            Float64, 
            f"/{self.robot_name}/{self.left_wheel_port}/setPosition", 
            10
        )
        self.pub_position_right = self.create_publisher(
            Float64, 
            f"/{self.robot_name}/{self.right_wheel_port}/setPosition", 
            10
        )

        self.send = False

    def set_next_movement(self):
        angle = self.counter * (math.pi / 2)

        msg = Float64()
        msg.data = angle

        if self.send:

            self.pub_position_left.publish(msg)
            self.pub_position_right.publish(msg)

        self.send = not self.send
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    set_position_reset_offset_node = SetPosition()

    rclpy.spin(set_position_reset_offset_node)

    set_position_reset_offset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()