import math
from os import closerange

from std_msgs.msg import Float32

from webots_ros2.base_plugin import BasePlugin

class CompassPlugin(BasePlugin):
    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.device.enable(int(self.timestep / 2))

        self.value_publisher = self.create_publisher(Float32, "compass", 10)

    def step(self):
        self.publish_angle()

    def publish_angle(self):
        """
            Publish the angle to the magnetic north in rad
            Starts at 0 for North, Clockwise to East, South, West and constantly increasing
            angle element of [0, 2pi)

            Magntic X is Y in Webots
            Therefore:
            - North -> + Y
            - East -> + X
            - South -> - Y
            - West -> - X
        """ 
        # Returns array of size 3 with [x, y, z] in ENU coordinates
        x, y, _ = self.device.getValues()

        # Calculate angle with the arctan cause tan(phi) = y / x
        # Which is not defined for x equals 0
        if x == 0:
            if y > 0:
                angle = math.pi / 2
            else:
                angle = 3 * math.pi / 2
        elif x > 0:
            angle = (math.atan(y / x) + 2 * math.pi) % (2 * math.pi)
        else:
            angle = math.atan(y / x) + math.pi
        
        msg = Float32()
        msg.data = angle

        self.value_publisher.publish(msg)