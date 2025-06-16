cmake_minimum_required(VERSION 3.0.2)
project(ur5e_haptic_teleop)

# 改动1: 将 omni_msgs 替换为 geomagic_control
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  geomagic_control 
  eigen_conversions
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(haptic_teleop_node src/haptic_teleop_node.cpp)

# 改动2: 添加此行以确保编译顺序正确
add_dependencies(haptic_teleop_node ${catkin_EXPORTED_TARGETS})

# 链接库保持不变，因为您的原始代码已使用eigen_conversions
target_link_libraries(haptic_teleop_node
  ${catkin_LIBRARIES}
)