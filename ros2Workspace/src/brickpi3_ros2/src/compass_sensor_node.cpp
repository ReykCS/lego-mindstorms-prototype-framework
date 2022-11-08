#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"

#include "../include/brickpi3_ros2/base_sensor_node.hpp"
#include "../include/brickpi3_ros2/compass_sensor_node.hpp"

#include "../libs/BrickPi3/BrickPi3.hpp"

#include "../include/brickpi3_ros2/constants.hpp"

using namespace std::chrono;
using namespace std;

CompassSensorNode::CompassSensorNode(): BaseSensorNode("compass") {
    this->publisher = this->create_publisher<std_msgs::msg::Float32>("compass", 10);

    this->sensor.address = 0x02;
    this->sensor.speed = 20;
    this->sensor.length_write = 1;
    this->sensor.buffer_write[0] = 0x42;
    this->sensor.length_read = 4;

    this->bp = BrickPi3();
    this->bp.set_sensor_type(this->port, SENSOR_TYPE_I2C, 0x0, &this->sensor);
}

void CompassSensorNode::publish_value_callback() {
    std_msgs::msg::Float32 message = std_msgs::msg::Float32();
    message.data = this->get_angle_from_read_buffer();

    this->publisher->publish(message);
}

int CompassSensorNode::read_value_from_brickpi3() {
    return this->bp.transact_i2c(this->port, &this->sensor);
}

float CompassSensorNode::get_angle_from_read_buffer() {
    this->read_value_from_brickpi3();

    this->bp.get_sensor(this->port, &this->sensor);

    int value = (
        ((this->sensor.buffer_read[3] & 0xFF) << 24)
        | ((this->sensor.buffer_read[2] & 0xFF) << 16)
        | ((this->sensor.buffer_read[1] & 0xFF) << 8)
        | (this->sensor.buffer_read[0] & 0xFF)
    );

    float angle = (((float) value) / ((float) (MAX_COMPASS_VALUE + COMPASS_INCREMENT))) * 2 * M_PI;

    return angle;

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CompassSensorNode>());
    rclcpp::shutdown();
    
    return 0;
}