cmake_minimum_required(VERSION 3.8)
project(s1_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build Rust library first
set(RUST_LIB_NAME "s1_driver_rust")
set(RUST_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(RUST_TARGET_DIR "${CMAKE_CURRENT_SOURCE_DIR}/target")

# Create custom command to build Rust library
add_custom_command(
    OUTPUT ${RUST_TARGET_DIR}/release/deps/lib${RUST_LIB_NAME}.a
    COMMAND cargo build --release --lib
    WORKING_DIRECTORY ${RUST_LIB_DIR}
    COMMENT "Building Rust library ${RUST_LIB_NAME}"
    VERBATIM
)

# Create custom target for Rust library
add_custom_target(rust_lib_build 
    DEPENDS ${RUST_TARGET_DIR}/release/deps/lib${RUST_LIB_NAME}.a
)

# Create the s1_driver executable
add_executable(s1_driver_node
  src/s1_driver_node.cpp
  src/s1_driver.cpp
  src/rust_bridge.cpp
)

# Explicitly set include directories for the target to ensure local headers are used
target_include_directories(s1_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Make sure Rust library is built first
add_dependencies(s1_driver_node rust_lib_build)

# Link Rust library
target_link_libraries(s1_driver_node
  ${RUST_TARGET_DIR}/release/deps/lib${RUST_LIB_NAME}.a
  pthread
  dl
  m
)

ament_target_dependencies(s1_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS
  s1_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
