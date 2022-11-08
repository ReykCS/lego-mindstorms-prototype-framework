#include <stdio.h>
#include <stdlib.h>

#include "../include/brickpi3_ros2/base_sensor_node.hpp"
#include "../include/brickpi3_ros2/constants.hpp"
#include "../include/brickpi3_ros2/utils.hpp"

using namespace std::chrono_literals;
using namespace std;

BaseSensorNode::BaseSensorNode(string name): Node(name) {
    this->port = utils::get_port_from_parameter(*this);

    chrono::milliseconds period_time = utils::get_duration_from_timestep_parameter(*this);

    this->timer = this->create_wall_timer(
        period_time, 
        bind(&BaseSensorNode::publish_value_callback, this)
    );
}

void BaseSensorNode::read_value_from_sensor() {
    if ( this->read_value_from_brickpi3() ) {
        RCLCPP_ERROR(this->get_logger(), "Get value error");
        exit(-2);
    }
}