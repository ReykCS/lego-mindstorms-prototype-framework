#ifndef COMPASS_SENSOR_NODE_HPP
#define COMPASS_SENSOR_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "../../libs/BrickPi3/BrickPi3.hpp"

#include "base_sensor_node.hpp"

using namespace std::chrono_literals;

class CompassSensorNode: public BaseSensorNode {
    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
        i2c_struct_t sensor;

    public:
        CompassSensorNode();

        void publish_value_callback() override;
        int read_value_from_brickpi3() override;

        float get_angle_from_read_buffer();
};

#endif