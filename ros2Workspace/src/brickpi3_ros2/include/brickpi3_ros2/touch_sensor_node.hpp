#ifndef TOUCH_SENSOR_NODE_HPP
#define TOUCH_SENSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "../../libs/BrickPi3/BrickPi3.hpp"

#include "base_sensor_node.hpp"

using namespace std::chrono_literals;

class TouchSensorNode: public BaseSensorNode {
    private:
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
        sensor_touch_t sensor;

    public:
        TouchSensorNode();

        void publish_value_callback() override;
        int read_value_from_brickpi3() override;
};

#endif