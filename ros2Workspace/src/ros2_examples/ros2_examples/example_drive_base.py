from rclpy.node import Node

from std_msgs.msg import Empty

"""
Set Power Example

Used robot model: CompleteRobot
"""

class DriveExampleNode(Node):
    def __init__(self, name, print_movement=True):
        super().__init__(name)

        if print_movement: 
            print(
                """
                The robot should loop through following movements, for each movement a new random value is selected:
                    - Drive forwards 
                    - Turn left
                    - Drive forwards
                    - Turn left
                    - Turn Right
                    - Drive backwards
                    - Turn right
                    - Turn right
                    - Turn right
                    - Drive forwards
                    - Stop
                """
            )

        self.left_wheel_port = "PORT_B"
        self.right_wheel_port = "PORT_A"

        self.movements = [
            self.forwards,
            self.left,
            self.left,
            self.right,
            self.backwards,
            self.right,
            self.right,
            self.right,
            self.forwards,
            self.stop
        ]

        self.declare_parameter("robot_name")
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        self.counter = 0

        self.waiting = 0

        self.should_send = False

        self.create_subscription(Empty, "/webots/step", self.webots_step_callback, 10)

    def webots_step_callback(self, _):
        self.set_next_movement()

    def set_next_movement(self):
        random_value = self.get_random_movement_value()

        self.waiting -= 1

        if self.should_send and self.waiting <= 0:
            print(f"{self.counter + 1} / {len(self.movements)}")
            self.movements[self.counter](random_value)
            self.waiting = 100
            self.counter = (self.counter + 1) % len(self.movements)

        self.should_send = not self.should_send

    def get_random_movement_value(self):
        raise NotImplementedError()

    def forwards(self, value):
        print("Forwards\n")
        # Left turns clockwise (forwards) -> positiv
        # Right turns clockwise (forwards) -> positiv
        self.send_new_movement(value, value)

    def left(self, value):
        print("Left\n")
        # Left turns counter clockwise (backwards) -> negative
        # Right turns clockwise (forwards) -> positive
        self.send_new_movement(-value, value)

    def right(self, value):
        print("Right\n")
        # Left turns clockwise (forwards) -> positiv
        # Right turns counter clockwise (backwards) -> negative
        self.send_new_movement(value, -value)

    def backwards(self, value):
        print("Backwards\n")
        # Left turns counter clockwise (backwards) -> negativ
        # Right turns counter clockwise (backwards) -> negative
        self.send_new_movement(-value, -value)

    def stop(self, _):
        print("Stopping\n")
        self.send_new_movement(0, 0)

    def send_new_movement(self, left_val, right_val):
        raise NotImplementedError()