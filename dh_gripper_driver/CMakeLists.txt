cmake_minimum_required(VERSION 3.8)
project(dh_gripper_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages only (no actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCtrl.msg"
  "msg/GripperState.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Include directories
include_directories(include)

# Create driver executable only
add_executable(dh_ag95_driver src/dh_ag95_driver.cpp)

ament_target_dependencies(dh_ag95_driver
  rclcpp
  std_msgs
  sensor_msgs
)

# Link with generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dh_ag95_driver "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  dh_ag95_driver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package() 