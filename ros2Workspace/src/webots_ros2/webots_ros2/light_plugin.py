from webots_ros2.constants import ColorWeight
import rclpy
from scipy.interpolate import interp1d

from std_msgs.msg import Int16

from webots_ros2.base_plugin import BasePlugin

class LightPlugin(BasePlugin):
    SENSOR_BLACK = 3200
    SENSOR_WHITE = 1600

    map_color_to_sensor_value = interp1d([0, 1], [SENSOR_BLACK, SENSOR_WHITE])

    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.device.enable(int(self.timestep / 2))

        self.width = self.device.getWidth()
        self.height = self.device.getHeight()

        self.size = float(self.width * self.height)

        self.value_publisher = self.create_publisher(Int16, "light", 10)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        self.publish_light()

    def get_weighted_color_value(self, color):
        r, g, b = color

        return (ColorWeight.RED * r + ColorWeight.GREEN * g + ColorWeight.BLUE * b) / 255

    def publish_light(self):
        image = self.device.getImageArray()

        r_sum, g_sum, b_sum = 0.0, 0.0, 0.0 

        for row in image:
            for val in row:
                r, g, b = val

                r_sum += float(r)
                g_sum += float(g)
                b_sum += float(b)

        color_mean = [i / self.size for i in [r_sum, g_sum, b_sum]]

        weighted_color_value = self.get_weighted_color_value(color_mean)

        sensor_value = int(LightPlugin.map_color_to_sensor_value(weighted_color_value))

        msg = Int16()
        msg.data = sensor_value

        self.value_publisher.publish(msg)