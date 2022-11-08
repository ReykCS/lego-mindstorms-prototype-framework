#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"

#include "../include/brickpi3_ros2/constants.hpp"
#include "../include/brickpi3_ros2/clock.hpp"
#include "../include/brickpi3_ros2/utils.hpp"

using namespace std::chrono;
using namespace std;

Clock::Clock(): Node("clock_node") {
    this->publisher = this->create_publisher<std_msgs::msg::Empty>("step", 10);

    chrono::milliseconds period_time = utils::get_duration_from_timestep_parameter(*this);

    this->timer = this->create_wall_timer(
        period_time, 
        bind(&Clock::step_callback, this)
    );
}

void Clock::step_callback() {
    std_msgs::msg::Empty msg = std_msgs::msg::Empty();

    this->publisher->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Clock>());
    rclcpp::shutdown();

    return 0;
}