#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "../include/brickpi3_ros2/utils.hpp"
#include "../include/brickpi3_ros2/constants.hpp"

using namespace std::chrono_literals;

std::chrono::milliseconds utils::get_duration_from_timestep(int timestep) {
    // Timestep is in milliseconds
    return std::chrono::milliseconds(timestep);
}

std::chrono::milliseconds utils::get_duration_from_timestep_parameter(rclcpp::Node &node) {
    node.declare_parameter<int16_t>(TIMESTEP, -1);

    int timestep;
    node.get_parameter(TIMESTEP, timestep);

    if ( timestep == -1 ) {
        RCLCPP_ERROR(node.get_logger(), "Timestep [ms] parameter is not set.");
        exit(1);
    }

    return utils::get_duration_from_timestep(timestep);
}

int8_t utils::get_port_from_parameter(rclcpp::Node &node) {
    // PORT_1 0x01
    // PORT_2 0x02
    // PORT_3 0x04
    // PORT_4 0x08

    node.declare_parameter<int8_t>(PORT_NAME, -1);

    int8_t port; 
    node.get_parameter(PORT_NAME, port);

    if ( port == -1 ) {
        RCLCPP_ERROR(node.get_logger(), "Port parameter is not set.");
        exit(2);
    }

    return port;
}

float utils::degree_to_radians(int32_t degree) {
    return (float) degree * (M_PI / 180.0);
}

int32_t utils::radians_to_degree(float radians) {
    return (int32_t) (radians * 180 / M_PI);
}