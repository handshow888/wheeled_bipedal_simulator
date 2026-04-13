cmake_minimum_required(VERSION 3.8)
project(wheeled_bipedal_hardware)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

include_directories(include)

add_executable(serial_test src/serial_test.cpp)
ament_target_dependencies(serial_test
  rclcpp
)

# add_library(${PROJECT_NAME} SHARED
#   src/mobile_base_hardware_interface.cpp
#   src/arm_hardware_interface.cpp
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
)

# pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
)

ament_package()
