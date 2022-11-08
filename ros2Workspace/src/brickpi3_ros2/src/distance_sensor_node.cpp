#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int16.hpp"

#include "../include/brickpi3_ros2/base_sensor_node.hpp"
#include "../include/brickpi3_ros2/distance_sensor_node.hpp"

#include "../libs/BrickPi3/BrickPi3.hpp"

using namespace std::chrono;
using namespace std;

DistanceSensorNode::DistanceSensorNode(): BaseSensorNode("distance") {
    this->publisher = this->create_publisher<std_msgs::msg::Int16>("distance", 10);

    this->bp.set_sensor_type(this->port, SENSOR_TYPE_NXT_ULTRASONIC);
}

void DistanceSensorNode::publish_value_callback() {
    std_msgs::msg::Int16 message = std_msgs::msg::Int16();
    this->read_value_from_brickpi3();

    message.data = this->sensor.cm;

    this->publisher->publish(message);
}

int DistanceSensorNode::read_value_from_brickpi3() {
    return this->bp.get_sensor(this->port, &this->sensor);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DistanceSensorNode>());
    rclcpp::shutdown();
    
    return 0;
}