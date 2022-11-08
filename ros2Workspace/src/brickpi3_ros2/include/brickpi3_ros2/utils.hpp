#ifndef UTILS_HPP
#define UTILS_HPP

#include "rclcpp/rclcpp.hpp"

namespace utils {
    std::chrono::milliseconds get_duration_from_timestep(int timestep);

    std::chrono::milliseconds get_duration_from_timestep_parameter(rclcpp::Node &node);
    
    int8_t get_port_from_parameter(rclcpp::Node &node);

    float degree_to_radians(int32_t degrees);
    int32_t radians_to_degree(float radians);

    template <typename T> 
    T clamp_to_limit(T value, T limit) {
        if ( value > limit ) {
            return limit;
        }

        if ( value < -limit ) {
            return -limit;
        }

        return value;
    }
}

#endif