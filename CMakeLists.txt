cmake_minimum_required(VERSION 3.5)
project(cylinder_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cylinder_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(cylinder_status
  src/cylinder_status.cpp
  )
ament_target_dependencies(cylinder_status
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  )

add_executable(imu2pose
  src/imu2pose.cpp
  )
ament_target_dependencies(imu2pose
  rclcpp
  geometry_msgs
  sensor_msgs
  )

add_executable(reading_laser
  src/reading_laser.cpp
  )
ament_target_dependencies(reading_laser
  rclcpp
  sensor_msgs
  )

add_executable(moving_robot
  src/moving_robot.cpp
  )
ament_target_dependencies(moving_robot
  rclcpp
  geometry_msgs
  )

add_executable(obstacle_avoidance
  src/obstacle_avoidance.cpp
  )
ament_target_dependencies(obstacle_avoidance
  rclcpp
  geometry_msgs
  sensor_msgs
  )

add_library(${PROJECT_NAME} SHARED
  src/cylinder_controller.cpp
  src/cylinder_move.cpp
  )
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  cylinder_interfaces
  )
rclcpp_components_register_nodes(${PROJECT_NAME} "cylinder_ros2::CylinderController")
rclcpp_components_register_nodes(${PROJECT_NAME} "cylinder_ros2::CylinderMove")
  
add_executable(apriltag_detect
  src/apriltag_detect.cpp
  )
ament_target_dependencies(apriltag_detect
  rclcpp
  cv_bridge
  sensor_msgs
  apriltag_msgs
  message_filters
  )

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

install(TARGETS
  cylinder_status
  imu2pose
  reading_laser
  moving_robot
  obstacle_avoidance
  apriltag_detect
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
