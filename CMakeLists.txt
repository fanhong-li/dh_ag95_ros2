cmake_minimum_required(VERSION 3.8)
project(dh_ag95_gripper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCtrl.msg"
  "msg/GripperState.msg"
  "action/GripperCommand.action"
  DEPENDENCIES std_msgs sensor_msgs
)

# Include directories
include_directories(include)

# Create driver executable
add_executable(dh_ag95_driver src/dh_ag95_driver.cpp)

ament_target_dependencies(dh_ag95_driver
  rclcpp
  std_msgs
  sensor_msgs
)

# Link with generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dh_ag95_driver "${cpp_typesupport_target}")

# Create action server executable
add_executable(gripper_action_server src/gripper_action_server.cpp)

ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
)

target_link_libraries(gripper_action_server "${cpp_typesupport_target}")

# Create action client example executable
add_executable(gripper_action_client_example src/gripper_action_client_example.cpp)

ament_target_dependencies(gripper_action_client_example
  rclcpp
  rclcpp_action
)

target_link_libraries(gripper_action_client_example "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  dh_ag95_driver
  gripper_action_server
  gripper_action_client_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

# Install meshes
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package() 