cmake_minimum_required(VERSION 3.5)
project(kridtbot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 

# Define the executable
add_executable(linear_position_pub nodes/linear_position_pub.cpp)

# Link the necessary dependencies
ament_target_dependencies(linear_position_pub rclcpp std_msgs sensor_msgs)

# Include directories (Force CMake to find the correct headers)
include_directories(/opt/ros/jazzy/include)

# Install the executable
install(TARGETS linear_position_pub DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config description launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
