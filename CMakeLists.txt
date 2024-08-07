cmake_minimum_required(VERSION 3.5)
project(livox_laser_simulation)

## Compile as C++17, supported in ROS2
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(include)

# To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-11/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

# Install launch files.
install(DIRECTORY
  launch
  worlds
  meshes
  urdf
  scan_mode
  DESTINATION share/${PROJECT_NAME}/
)

# Add the library for the plugin
add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)

# Link against the necessary libraries
ament_target_dependencies(livox_laser_simulation 
  ament_cmake
  rclcpp 
  rclcpp_components 
  tf2 
  tf2_ros 
  gazebo_ros
  sensor_msgs
)

# Link against Gazebo libraries
target_link_libraries(livox_laser_simulation RayPlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
