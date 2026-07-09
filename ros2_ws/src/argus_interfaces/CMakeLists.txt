cmake_minimum_required(VERSION 3.8)
project(argus_interfaces)

# --- COMPILATION OPTIONS ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# --- MESSAGE GENERATION ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlcCommand.msg"
  "msg/PlcStatus.msg"
  "msg/TeleopCommand.msg"
  "msg/ManagerStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# --- EXPORT ---
ament_package()
