cmake_minimum_required(VERSION 3.5)
project(altosradar)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # For message generation

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YourMessage.msg"  # Replace with actual message definitions if any
  DEPENDENCIES std_msgs  # Add other dependencies as needed
)

# Include directories
include_directories(
  include
)

# Create the executable
add_executable(altosRadarParse src/altosRadarParse.cpp)

# Link against ROS2 libraries
ament_target_dependencies(altosRadarParse
  rclcpp
  sensor_msgs
  std_msgs
  pcl_ros
)

# Install the executable
install(TARGETS altosRadarParse
  DESTINATION lib/${PROJECT_NAME})

# Export the package
ament_package()
