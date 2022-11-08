#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "../include/brickpi3_ros2/base_sensor_node.hpp"
#include "../include/brickpi3_ros2/touch_sensor_node.hpp"

#include "../libs/BrickPi3/BrickPi3.hpp"

using namespace std::chrono;
using namespace std;

TouchSensorNode::TouchSensorNode(): BaseSensorNode("touch") {
    this->publisher = this->create_publisher<std_msgs::msg::Bool>("touch", 10);

    this->bp.set_sensor_type(this->port, SENSOR_TYPE_TOUCH_NXT);
}

void TouchSensorNode::publish_value_callback() {
    std_msgs::msg::Bool message = std_msgs::msg::Bool();
    message.data = this->read_value_from_brickpi3() == 1 ? true : false;

    this->publisher->publish(message);
}

int TouchSensorNode::read_value_from_brickpi3() {
    return this->bp.get_sensor(this->port, &this->sensor);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TouchSensorNode>());
    rclcpp::shutdown();
    
    return 0;
}