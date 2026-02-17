cmake_minimum_required(VERSION 3.8)
project(argus_joystick_driver)

# --- COMPILATION OPTIONS ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(SFML COMPONENTS window system REQUIRED)

# --- EXECUTABLES ---
add_executable(joystick_driver_node src/joystick_driver_node.cpp)

# --- INCLUDE AND ROS 2 DEPENDENCIES ---
target_include_directories(joystick_driver_node PRIVATE ${SFML_INCLUDE_DIR})

ament_target_dependencies(joystick_driver_node
  rclcpp
  sensor_msgs
)

# --- EXTERNAL LIBRARIES LINKING ---
target_link_libraries(joystick_driver_node sfml-window sfml-system)

# --- INSTALLATION ---
install(TARGETS joystick_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- EXPORT ---
ament_package()
