cmake_minimum_required(VERSION 3.0.2)
project(ur5e_haptic_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  trajectory_msgs
  geomagic_control
  moveit_core
  moveit_ros_planning_interface
  tf2_ros
  sensor_msgs
  tf2_geometry_msgs
  xacro
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    trajectory_msgs
    geomagic_control
    moveit_core
    moveit_ros_planning_interface
    tf2_ros
    sensor_msgs
    tf2_geometry_msgs
    xacro
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(haptic_teleop_node src/haptic_teleop_node.cpp)

add_dependencies(haptic_teleop_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(haptic_teleop_node
  ${catkin_LIBRARIES}
)