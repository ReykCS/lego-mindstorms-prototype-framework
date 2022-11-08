#ifndef BASE_SENSOR_NODE_HPP
#define BASE_SENSOR_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "../../libs/BrickPi3/BrickPi3.hpp"

class BaseSensorNode: public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer;

    protected:
        BrickPi3 bp;
        int8_t port;

    public:
        BaseSensorNode(std::string name);

        virtual void publish_value_callback() {};

        // Returned value by brickpi is always an int
        void read_value_from_sensor();

        virtual int read_value_from_brickpi3() {
            return 0;
        };
};

#endif