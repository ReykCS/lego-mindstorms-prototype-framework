#include <memory>
#include <vector>

#include "../include/brickpi3_ros2/motor_node.hpp"
#include "../include/brickpi3_ros2/utils.hpp"
#include "../include/brickpi3_ros2/constants.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/empty.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std;
using std::placeholders::_1;

MotorNode::MotorNode(): Node("motor") {
    // init publishers
    this->position_publisher = this->create_publisher<std_msgs::msg::Float64>("position", 10);
    this->velocity_publisher = this->create_publisher<std_msgs::msg::Float32>("velocity", 10);
    this->power_publisher = this->create_publisher<std_msgs::msg::Int16>("power", 10);

    this->flags_publisher = this->create_publisher<std_msgs::msg::Int16>("flags", 10);

    this->power_limit_publisher = this->create_publisher<std_msgs::msg::Int16>("powerLimit", 10);
    this->velocity_limit_publisher = this->create_publisher<std_msgs::msg::Float32>("velocityLimit", 10);

    // init subscribers
    this->set_velocity_subscriber = this->create_subscription<std_msgs::msg::Float32>(
        "setVelocity",
        10,
        bind(&MotorNode::set_velocity_callback, this, _1)
    );
    this->set_position_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "setPosition", 
        10,
        bind(&MotorNode::set_position_callback, this, _1)
    );
    this->set_power_subscriber = this->create_subscription<std_msgs::msg::Int16>(
        "setPower", 
        10,
        bind(&MotorNode::set_power_callback, this, _1)
    );

    this->set_position_offset_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "setPositionOffset", 
        10,
        bind(&MotorNode::set_position_offset_callback, this, _1)
    );
    this->reset_position_offset_subscriber = this->create_subscription<std_msgs::msg::Empty>(
        "resetPositionOffset", 
        10,
        bind(&MotorNode::reset_position_offset_callback, this, _1)
    );
    
    this->set_velocity_limit_subscriber = this->create_subscription<std_msgs::msg::Float32>(
        "setVelocityLimit", 
        10,
        bind(&MotorNode::set_velocity_limit_callback, this, _1)
    );
    this->set_power_limit_subscriber = this->create_subscription<std_msgs::msg::Int16>(
        "setPowerLimit", 
        10,
        bind(&MotorNode::set_power_limit_callback, this, _1)
    );

    // Get port and timestep
    this->port = utils::get_port_from_parameter(*this);
    chrono::milliseconds timestep = utils::get_duration_from_timestep_parameter(*this);

    // Set timer for publishing
    this->timer = this->create_wall_timer(
        timestep, 
        bind(&MotorNode::publish_values_callback, this)
    );

    // Set up Brickpi
    this->bp.detect();
    this->bp.reset_motor_encoder(this->port);

    // Set limits
    this->power_limit = POW_LIMIT;
    this->velocity_limit = VEL_LIMIT;
}

// Publishers

void MotorNode::publish_values_callback() {
    uint8_t state;
    int8_t power;
    int32_t position; // in degrees
    int16_t dps; // degrees per second

    this->bp.get_motor_status(this->port, state, power, position, dps);

    this->publish_position(position);
    this->publish_velocity(dps);
    this->publish_power(power);

    this->publish_power_limit();
    this->publish_velocity_limit();
}

void MotorNode::publish_position(int32_t position) {
    std_msgs::msg::Float64 message = std_msgs::msg::Float64();
    message.data = utils::degree_to_radians(position);

    this->position_publisher->publish(message);
}

void MotorNode::publish_velocity(int16_t velocity) {
    std_msgs::msg::Float32 message = std_msgs::msg::Float32();
    message.data = utils::degree_to_radians(velocity);

    this->velocity_publisher->publish(message);
}

void MotorNode::publish_power(int8_t power) {
    std_msgs::msg::Int16 message = std_msgs::msg::Int16();
    message.data = power;

    this->power_publisher->publish(message);
}

void MotorNode::publish_power_limit() {
    std_msgs::msg::Int16 message = std_msgs::msg::Int16();
    message.data = this->power_limit;

    this->power_limit_publisher->publish(message);
}

void MotorNode::publish_velocity_limit() {
    std_msgs::msg::Float32 message = std_msgs::msg::Float32();
    message.data = this->velocity_limit;

    this->velocity_limit_publisher->publish(message);
}

// Subscribers

void MotorNode::set_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    float new_velocity = msg->data;

    float clamped_values = utils::clamp_to_limit<float>(new_velocity, this->velocity_limit);
    int16_t velocity_in_degrees = (int16_t) utils::radians_to_degree(clamped_values);

    this->bp.set_motor_dps(this->port, velocity_in_degrees);
}

void MotorNode::set_power_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    int16_t new_power = msg->data;

    int16_t clamped_power = utils::clamp_to_limit(new_power, this->power_limit);

    this->bp.set_motor_power(this->port, clamped_power);
}

void MotorNode::set_position_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    float new_position = msg->data;

    int32_t position_in_degrees = (int32_t) utils::radians_to_degree(new_position);

    this->bp.set_motor_position(this->port, position_in_degrees);
}

void MotorNode::set_position_offset_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    float position = msg->data;

    int32_t position_in_degrees = (int32_t) utils::radians_to_degree(position);

    this->bp.offset_motor_encoder(this->port, position_in_degrees);
}

void MotorNode::reset_position_offset_callback(const std_msgs::msg::Empty::SharedPtr) {
    this->bp.reset_motor_encoder(this->port);
}

void MotorNode::set_velocity_limit_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    float new_velocity_limit = msg->data;

    float max_limit = (float) VEL_LIMIT;
    float clamped_velocity_limit = utils::clamp_to_limit(new_velocity_limit, max_limit);

    this->velocity_limit = clamped_velocity_limit;
    this->bp.set_motor_limits(this->port, this->power_limit, this->velocity_limit);
}

void MotorNode::set_power_limit_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    float new_power_limit = msg->data;

    float max_limit = (float) POW_LIMIT;
    float clamped_power_limit = utils::clamp_to_limit(new_power_limit, max_limit);

    this->power_limit = clamped_power_limit;
    this->bp.set_motor_limits(this->port, this->power_limit, this->velocity_limit);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    
    return 0;
}