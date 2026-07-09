cmake_minimum_required(VERSION 3.8)
project(argus_teleop_keyboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(argus_interfaces REQUIRED)

add_executable(teleop_keyboard_node src/teleop_keyboard_node.cpp)
ament_target_dependencies(teleop_keyboard_node rclcpp argus_interfaces)

install(TARGETS teleop_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
