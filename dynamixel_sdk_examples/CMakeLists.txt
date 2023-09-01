cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk_examples)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
# added by katayama
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# Build
add_executable(read_write_node src/read_write_node.cpp)
ament_target_dependencies(read_write_node
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
)
# added by katayama
add_executable(read_write_node2 src/read_write_node2.cpp)
ament_target_dependencies(read_write_node2
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
  geometry_msgs
)

# added by katayama
add_executable(publish_wheel_odometry src/publish_wheel_odometry.cpp)
ament_target_dependencies(publish_wheel_odometry
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Install
install(TARGETS
  read_write_node
  read_write_node2
  publish_wheel_odometry
  DESTINATION lib/${PROJECT_NAME}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
