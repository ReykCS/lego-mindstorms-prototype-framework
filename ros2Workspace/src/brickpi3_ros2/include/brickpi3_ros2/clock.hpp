#ifndef CLOCK_NODE_HPP
#define CLOCK_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

class Clock: public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher;

    public:
        Clock();

        void step_callback();
};

#endif