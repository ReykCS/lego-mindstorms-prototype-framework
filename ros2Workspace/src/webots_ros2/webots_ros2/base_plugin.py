import rclpy
from rclpy.node import Node

class BasePlugin:
    def init(self, webots_node, properties):
        try: 
            rclpy.init(args=None)
        except:
            pass

        self.port = properties["port"]
        self.robot_name = properties["robotName"]

        self.node = Node(self.robot_name + self.port)
        
        self.robot = webots_node.robot
        self.device = self.robot.getDevice(self.port)
        self.timestep = int(self.robot.getBasicTimeStep()) # in ms

        self.with_namespace = lambda topic: f"{self.robot_name}/{self.port}/{topic}"

    def create_publisher(self, type, topic, qos=10):
        return self.node.create_publisher(type, self.with_namespace(topic), qos)

    def create_service(self, type, topic, callback):
        return self.node.create_service(type, self.with_namespace(topic), callback)

    def create_subscription(self, type, topic, callback, qos=10):
        return self.node.create_subscription(type, self.with_namespace(topic), callback, qos)