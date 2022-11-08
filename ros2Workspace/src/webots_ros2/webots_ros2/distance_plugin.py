import rclpy

from std_msgs.msg import Int16

from webots_ros2.base_plugin import BasePlugin

class DistancePlugin(BasePlugin):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.device.enable(int(self.timestep / 2))

        self.value_publisher = self.create_publisher(Int16, "distance", 10)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        self.publish_distance()

    def publish_distance(self):
        distance = self.device.getValue()

        msg = Int16()
        msg.data = int(distance * 100)

        self.value_publisher.publish(msg)