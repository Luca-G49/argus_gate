cmake_minimum_required(VERSION 3.8)
project(argus_teleop_joy)

# --- COMPILATION OPTIONS ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(argus_interfaces REQUIRED)

# --- EXECUTABLES ---
add_executable(argus_teleop_joy_node src/argus_teleop_joy_node.cpp)

# --- INCLUDE AND ROS 2 DEPENDENCIES ---
ament_target_dependencies(argus_teleop_joy_node
  rclcpp
  sensor_msgs
  argus_interfaces
)

# --- INSTALLATION ---
install(TARGETS argus_teleop_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- EXPORT ---
ament_package()
