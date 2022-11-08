import math

from webots_ros2.utils import Limit

class Velocity:
    # Velocity is in rad/s
    # 5 * PI = 15.707
    # equals 2.5 rotations per second
    # means 900 degrees
    MAX = 5 * math.pi
    MIN = -5 * math.pi

    @staticmethod
    def get_velocity_limits():
        return Limit(Velocity.MIN, Velocity.MAX)

class Power:
    # Power level for one direction can either be in range 0 to 100
    # Though, the motor can turn forwards and backwards the real limits
    # are -100 and 100
    MAX = 100
    MIN = -100

    @staticmethod
    def get_power_limit():
        return Limit(Power.MIN, Power.MAX)

class ColorWeight:
    RED = 0.6
    GREEN = 0.3
    BLUE = 0.1