cmake_minimum_required(VERSION 3.5)
project(smoothop_interfaces)

# Default to C++14 if no language is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # Make sure std_msgs is found

# Specify the message and service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorMsg.msg"
  "msg/WheelSpeed.msg"
  DEPENDENCIES std_msgs  # Add std_msgs here as a dependency
)

# Install the generated files
ament_package()
