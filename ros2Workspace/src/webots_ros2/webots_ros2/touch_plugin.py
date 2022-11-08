from std_msgs.msg import Bool

from webots_ros2.base_plugin import BasePlugin

class TouchPlugin(BasePlugin):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.device.enable(int(self.timestep / 2))

        self.value_publisher = self.create_publisher(Bool, "touch", 10)

    def step(self):
        self.publish_is_pressed()

    def publish_is_pressed(self):
        is_pressed = self.device.getValue()
        
        msg = Bool()
        msg.data = True if is_pressed > 0 else False

        self.value_publisher.publish(msg)