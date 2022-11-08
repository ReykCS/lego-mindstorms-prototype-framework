from std_msgs.msg import Float64, Float32, Int16, Int16MultiArray, Float32MultiArray

class MotorPublisher:
    def __init__(self, node):
        pub = lambda type, topic: node.create_publisher(type, topic)

        self.position = pub(Float64, "position")
        self.velocity = pub(Float32, "velocity")
        self.power = pub(Int16, "power")
        self.flags = pub(Int16, "flags")

        self.power_limit = pub(Int16MultiArray, "powerLimit")
        self.velocity_limit = pub(Float32MultiArray, "velocityLimit")

    def publish_position(self, position):
        MotorPublisher.publish(self.position, Float64, position)

    def publish_velocity(self, velocity):
        MotorPublisher.publish(self.velocity, Float32, velocity)

    def publish_power(self, power):
        MotorPublisher.publish(self.power, Int16, power)

    def publish_flags(self, flags=0):
        MotorPublisher.publish(self.flags, Int16, flags)

    def publish_power_limit(self, power_limit):
        data = [int(power_limit.min), int(power_limit.max)]

        MotorPublisher.publish(self.power_limit, Int16MultiArray, data)

    def publish_velocity_limit(self, velocity_limit):
        data = [float(velocity_limit.min), float(velocity_limit.max)]

        MotorPublisher.publish(self.velocity_limit, Float32MultiArray, data)

    def publish_values(self, position, velocity, power, flags=0):
        self.publish_position(position)
        self.publish_velocity(velocity)
        self.publish_power(power)
        self.publish_flags(flags)

    def publish_limits(self, power_limit, velocity_limit):
        self.publish_power_limit(power_limit)
        self.publish_velocity_limit(velocity_limit)

    @staticmethod
    def publish(pipe, msg_type, data):
        msg = msg_type()
        msg.data = data

        pipe.publish(msg)