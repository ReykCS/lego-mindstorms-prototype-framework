import rclpy

from rclpy.node import Node

from std_msgs.msg import Empty

class ClockPlugin:
    def init(self, webots_node, properties):
        try: 
            rclpy.init(args=None)
        except:
            pass

        self.robot_name = properties["robotName"]

        self.node = Node(self.robot_name + "_clock")

        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep()) # in ms

        self.clock_publisher = self.node.create_publisher(Empty, "/step", 10)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        self.clock_publisher.publish(Empty())
