cmake_minimum_required(VERSION 3.8)
project(argus_joystick)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(argus_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find SFML
find_package(SFML COMPONENTS window system REQUIRED)

add_executable(joystick_node src/joystick_node.cpp)

target_include_directories(joystick_node PRIVATE ${SFML_INCLUDE_DIR})

ament_target_dependencies(joystick_node
  rclcpp
  sensor_msgs
)

# Link explicit system libraries
target_link_libraries(joystick_node sfml-window sfml-system)

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
