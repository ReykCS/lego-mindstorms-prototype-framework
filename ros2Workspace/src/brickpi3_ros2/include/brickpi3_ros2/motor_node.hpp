#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <string.h>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"

#include "constants.hpp"

#include "../../libs/BrickPi3/BrickPi3.hpp"

class MotorNode: public rclcpp::Node {
    private:
        BrickPi3 bp;

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr power_publisher;

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr flags_publisher;

        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr power_limit_publisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_limit_publisher;

        void publish_values_callback();

        void publish_position(int32_t position);
        void publish_velocity(int16_t velocity);
        void publish_power(int8_t power);
        void publish_flags();
        void publish_power_limit();
        void publish_velocity_limit();

        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_velocity_subscriber;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr set_power_subscriber;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_position_subscriber;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_position_offset_subscriber;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_position_offset_subscriber;
        
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_velocity_limit_subscriber;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr set_power_limit_subscriber;

        void set_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void set_power_callback(const std_msgs::msg::Int16::SharedPtr msg);
        void set_position_callback(const std_msgs::msg::Float64::SharedPtr msg);

        void set_position_offset_callback(const std_msgs::msg::Float64::SharedPtr msg);
        void reset_position_offset_callback(const std_msgs::msg::Empty::SharedPtr msg);

        void set_velocity_limit_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void set_power_limit_callback(const std_msgs::msg::Int16::SharedPtr msg);

        // Get params
        int16_t port;

        rclcpp::TimerBase::SharedPtr timer;

        int16_t power_limit; 
        float velocity_limit;

    public:
        MotorNode();
};

#endif