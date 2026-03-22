cmake_minimum_required(VERSION 3.8)
project(wheeled_bipedal_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
# include(
#   ${EIGEN3_INCLUDE_DIRS}
# )

add_library(${PROJECT_NAME} SHARED
  src/wheeled_bipedal_controller.cpp
  src/ins_task.cpp
  src/kalmanFilter.cpp
  src/QuaternionEKF.cpp
  src/utils.cpp
  src/kinematics.cpp
  src/LQR.cpp
  src/VMC.cpp
  src/structural_params.cpp
  src/pid_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "controller_interface"
  "pluginlib"
  "rclcpp_lifecycle"
)

pluginlib_export_plugin_description_file(controller_interface wheeled_bipedal_controller_plugin.xml)

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
  controller_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()
