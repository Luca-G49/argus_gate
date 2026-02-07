cmake_minimum_required(VERSION 3.8)
project(argus_plc_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Trova ROS2 e messaggi
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(argus_msgs REQUIRED)

# 2. Trova SFML (per gli include path)
find_package(SFML COMPONENTS network system REQUIRED)

# 3. Definisci l'eseguibile
add_executable(plc_bridge_node src/plc_bridge_node.cpp)

# 4. Include e Dipendenze ROS2
target_include_directories(plc_bridge_node PRIVATE ${SFML_INCLUDE_DIR})
ament_target_dependencies(plc_bridge_node 
  rclcpp 
  argus_msgs
)

# 5. LINK FORZATO (Risolve gli undefined reference)
# Usiamo i nomi delle librerie di sistema di Ubuntu
target_link_libraries(plc_bridge_node sfml-network sfml-system)

# 6. Installazione
install(TARGETS plc_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
