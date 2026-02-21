# Argus Gate - argus_bringup/CMakeLists.txt
# Copyright (c) 2026, Luca-G49
# All rights reserved. Licensed under MIT License.

cmake_minimum_required(VERSION 3.8)
project(argus_bringup)

# --- COMPILATION OPTIONS ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- DEPENDENCIES ---
find_package(ament_cmake REQUIRED)

# --- TESTING ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- INSTALLATION ---
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- EXPORT ---
ament_package()
