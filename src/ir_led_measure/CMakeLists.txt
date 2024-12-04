cmake_minimum_required(VERSION 3.8)
project(ir_led_measure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
# "msg/LinktrackNode2.msg"
# "msg/LinktrackNodeframe2.msg"
"msg/Direct.msg"
)

# Declare the messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (nodes)
install(PROGRAMS
  scripts/camera_publisher.py
  scripts/mvsdk.py
  scripts/fusion.py
  scripts/camera_rosbag.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
