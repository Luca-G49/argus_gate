cmake_minimum_required(VERSION 3.8)
project(argus_plc_bridge)

# --- COMPILATION OPTIONS ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(argus_interfaces REQUIRED)
find_package(SFML COMPONENTS network system REQUIRED)

# --- EXECUTABLES ---
add_executable(plc_bridge_node src/plc_bridge_node.cpp)

# --- INCLUDE AND ROS 2 DEPENDENCIES ---
target_include_directories(plc_bridge_node PRIVATE ${SFML_INCLUDE_DIR})

ament_target_dependencies(plc_bridge_node
  rclcpp
  argus_interfaces
)

# --- EXTERNAL LIBRARIES LINKING ---
target_link_libraries(plc_bridge_node sfml-network sfml-system)

# --- INSTALLATION ---
install(TARGETS plc_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- EXPORT ---
ament_package()
