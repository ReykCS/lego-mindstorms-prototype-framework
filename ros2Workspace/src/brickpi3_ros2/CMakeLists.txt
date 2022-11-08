cmake_minimum_required(VERSION 3.5)
project(brickpi3_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(DEFAULT_SOURCES
    libs/BrickPi3/BrickPi3.cpp
    src/utils.cpp
)

set(SENSOR_SOURCES
    ${DEFAULT_SOURCES}
    src/base_sensor_node.cpp
)

add_executable(clock_node
    src/clock.cpp
    ${DEFAULT_SOURCES}
)

add_executable(motor_node
    src/motor_node.cpp
    ${DEFAULT_SOURCES}
)

add_executable(distance_sensor 
    src/distance_sensor_node.cpp
    ${SENSOR_SOURCES}
)

add_executable(light_sensor
    src/light_sensor_node.cpp
    ${SENSOR_SOURCES}
)

add_executable(touch_sensor
    src/touch_sensor_node.cpp
    ${SENSOR_SOURCES}
)

add_executable(compass_sensor
    src/compass_sensor_node.cpp
    ${SENSOR_SOURCES}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(motor_node rclcpp std_msgs)

ament_target_dependencies(distance_sensor rclcpp std_msgs)
ament_target_dependencies(light_sensor rclcpp std_msgs)
ament_target_dependencies(touch_sensor rclcpp std_msgs)
ament_target_dependencies(compass_sensor rclcpp std_msgs)

ament_target_dependencies(clock_node rclcpp std_msgs)

install(TARGETS
    motor_node

    distance_sensor
    light_sensor  
    touch_sensor
    compass_sensor

    clock_node
    
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
