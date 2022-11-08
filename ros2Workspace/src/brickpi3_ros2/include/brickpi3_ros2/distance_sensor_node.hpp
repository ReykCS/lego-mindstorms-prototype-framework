#ifndef DISTANCE_SENSOR_NODE_HPP
#define DISTANCE_SENSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

#include "../../libs/BrickPi3/BrickPi3.hpp"

#include "base_sensor_node.hpp"

using namespace std::chrono_literals;

class DistanceSensorNode: public BaseSensorNode {
    private:
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher;
        sensor_ultrasonic_t sensor;

    public:
        DistanceSensorNode();

        void publish_value_callback() override;
        int read_value_from_brickpi3() override;
};

#endif