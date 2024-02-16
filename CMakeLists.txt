cmake_minimum_required(VERSION 3.5)

project(object_detection_simulation)

# Fortress
if("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)

  message(STATUS "Using Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-gazebo5 QUIET)

  if(NOT ignition-gazebo5_FOUND)
    # Skip if Ignition not present
    message(WARNING "Ignition Gazebo 5 or 6 not found, proceeding without that simulator.")
    return()
  else()
    message(STATUS "Using Ignition Edifice")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

set(ament_dependencies
  rclpy

  tf2
  tf2_ros

  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs

  cv_bridge 

)

add_executable(teleop src/teleop.cpp)
ament_target_dependencies(teleop rclcpp geometry_msgs)

install(TARGETS teleop
DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
scripts/detection_node.py
scripts/move_to_goal_node.py
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  models
  worlds
  ign
  rviz
  scripts
  DESTINATION
  share/${PROJECT_NAME}/
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
  
  # Ref: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths 
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()