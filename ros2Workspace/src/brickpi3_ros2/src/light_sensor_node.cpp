#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int16.hpp"

#include "../include/brickpi3_ros2/base_sensor_node.hpp"
#include "../include/brickpi3_ros2/light_sensor_node.hpp"

#include "../libs/BrickPi3/BrickPi3.hpp"

using namespace std::chrono;
using namespace std;

LightSensorNode::LightSensorNode(): BaseSensorNode("light") {
    this->publisher = this->create_publisher<std_msgs::msg::Int16>("light", 10);

    this->bp.set_sensor_type((uint8_t) this->port, (uint8_t) SENSOR_TYPE_NXT_LIGHT_ON);
}

void LightSensorNode::publish_value_callback() {
    std_msgs::msg::Int16 message = std_msgs::msg::Int16();
    this->read_value_from_brickpi3();

    message.data = this->sensor.reflected;

    this->publisher->publish(message);
}

int LightSensorNode::read_value_from_brickpi3() {
    return this->bp.get_sensor((uint8_t) this->port, &this->sensor);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LightSensorNode>());
    rclcpp::shutdown();
    
    return 0;
}