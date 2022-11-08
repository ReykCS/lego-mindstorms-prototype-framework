from webots_ros2.motor_publisher import MotorPublisher
import numpy as np
import rclpy
from scipy.interpolate import interp1d

from std_srvs.srv import Empty

# from controller import Robot, Motor
from webots_ros2.constants import Velocity, Power

from std_msgs.msg import Float64, Float32, Int16, Int16MultiArray, Float32MultiArray, Empty

from webots_ros2.base_plugin import BasePlugin

class MotorPlugin(BasePlugin):
    map_power_to_velocity = (
        lambda val: float(interp1d([Power.MIN, Power.MAX], [Velocity.MIN, Velocity.MAX])(val))
    )
    map_velocity_to_power = (
        lambda val: int(interp1d([Velocity.MIN, Velocity.MAX], [Power.MIN, Power.MAX])(val))
    )

    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        self.create_services()

        self.set_velocity(0)

        self.device.setAcceleration(100)

        self.position_sensor = self.robot.getDevice(self.port + "_position")
        self.position_sensor.enable(int(self.timestep / 2))

        self.power_limit = Power.get_power_limit()
        self.velocity_limit = Velocity.get_velocity_limits()

        self.position_offset = 0.0

        self.publisher = MotorPublisher(self)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        real_velocity = self.device.getVelocity()

        self.publisher.publish_values(
            self.get_position(), 
            real_velocity, 
            int(MotorPlugin.map_velocity_to_power(real_velocity))
        )
        self.publisher.publish_limits(
            self.power_limit, 
            self.velocity_limit
        )

    def set_velocity(self, velocity):
        self.device.setVelocity(velocity)

    def set_endless_velocity(self, velocity):
        self.device.setPosition(float('inf'))

        self.set_velocity(velocity)

    def set_power(self, power):
        self.set_endless_velocity(MotorPlugin.map_power_to_velocity(power))

    """
        Position
    """
    def increment_position(self, velocity):
        return float(self.timestep) * 0.001 * velocity + self.position_sensor.getValue()

    def apply_position(self, position):
        self.device.setPosition(position)
        self.set_velocity(self.velocity_limit.max)

    def set_position_offset(self, new_offset):
        self.position_offset = self.position_sensor.getValue() + new_offset

    def reset_position_encoder(self):
        self.position_offset = self.position_sensor.getValue()

    def set_position(self, new_position):
        self.apply_position(new_position + self.position_offset)

    def set_relative_position(self, new_position):
        self.apply_position(new_position + self.position_sensor.getValue())

    def get_position(self):
        return self.position_sensor.getValue() - self.position_offset

    """
        Subscriber callbacks
    """
    def set_position_callback(self, msg):
        self.set_position(msg.data)

    def set_position_offset_callback(self, msg):
        self.set_position_offset(msg.data)

    def reset_motor_position_callback(self, _):
        self.reset_position_encoder()

    def set_velocity_callback(self, msg):
        new_velocity = msg.data
        
        if not self.velocity_limit.is_in_range(new_velocity):
            new_velocity = self.velocity_limit.norm_value_to_range(new_velocity)

        self.set_endless_velocity(new_velocity)

    def set_power_callback(self, msg):
        new_power = msg.data

        if not self.power_limit.is_in_range(new_power):
            new_power = self.power_limit.norm_value_to_range(new_power) 

        self.set_power(new_power)

    def set_velocity_limit_callback(self, msg):
        self.velocity_limit.set_new_limit(msg.data)

    def set_power_limit_callback(self, msg):
        self.power_limit.set_new_limit(msg.data)

    def create_services(self):
        sub = lambda type, name, callback: self.create_subscription(
            type,
            name,
            callback
        )

        # Equivalent to 'set_motor_dps' but requires rad/s instead of deg/s
        sub(Float32, "setVelocity", self.set_velocity_callback)
        
        # Equivalent to 'set_motor_power'
        sub(Int16, "setPower", self.set_power_callback)

        sub(Float64, "setPosition", self.set_position_callback)
        sub(Float64, "setPositionOffset", self.set_position_offset_callback)
        sub(Empty, "resetPositionOffset", self.reset_motor_position_callback)

        sub(Float32MultiArray, "setVelocityLimit", self.set_velocity_limit_callback)
        sub(Int16MultiArray, "setPowerLimit", self.set_power_limit_callback)