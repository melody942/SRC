# CMakeLists.txt (完整版 V2)

cmake_minimum_required(VERSION 3.0.2)
project(ur5e_haptic_teleop)

# 在组件列表中加入了 tf2_eigen
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  omni_msgs
  eigen_conversions
  tf2_geometry_msgs
  tf2_eigen # 这是解决问题的关键
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(haptic_teleop_node src/haptic_teleop_node.cpp)

# ${catkin_LIBRARIES} 现在将正确地包含 tf2_eigen 库的链接信息
target_link_libraries(haptic_teleop_node
  ${catkin_LIBRARIES}
)
