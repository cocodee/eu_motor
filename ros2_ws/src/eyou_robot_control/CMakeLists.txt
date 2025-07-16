cmake_minimum_required(VERSION 3.8)
project(eyou_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(xacro REQUIRED)

# Add the C++ library for the hardware interface
add_library(eyou_system_interface SHARED
  src/eyou_system_interface.cpp
)
target_include_directories(eyou_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
  # TODO: Add the include directory for your actual SDK here
  # /path/to/your/eyou_sdk/include
)
ament_target_dependencies(eyou_system_interface PUBLIC
  hardware_interface
  rclcpp
  pluginlib
  # TODO: Add the library for your actual SDK here
  # e.g., /path/to/your/eyou_sdk/lib/libeyou_sdk.so
  /usr/lib/libeu_motor.so
  /usr/lib/libeu_harmonic.so
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface eyou_system_interface.xml)

# Install the library
install(TARGETS eyou_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch, config, and URDF files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Register with ament
ament_package()
