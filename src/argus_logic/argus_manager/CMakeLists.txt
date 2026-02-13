cmake_minimum_required(VERSION 3.8)
project(argus_manager)

# --- COMPILATION OPTIONS ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(argus_interfaces REQUIRED)

# --- EXECUTABLES ---
add_executable(manager_node src/manager_node.cpp)

# --- INCLUDE AND ROS 2 DEPENDENCIES ---
# (No external libraries needed for this node)

ament_target_dependencies(manager_node
  rclcpp
  sensor_msgs
  argus_interfaces
)

# --- EXTERNAL LIBRARIES LINKING ---
# (No extra linking required)

# --- INSTALLATION ---
install(TARGETS manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- EXPORT ---
ament_package()
