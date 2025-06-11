cmake_minimum_required(VERSION 3.0.2)
project(geomagic_myhaptics)

find_package(catkin REQUIRED COMPONENTS
  geomagic_control
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES geomagic_myhaptics
#  CATKIN_DEPENDS geomagic_control geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  # 这里取消注释
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pubforce src/pubforce.cpp)
add_dependencies(pubforce ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pubforce
  ${catkin_LIBRARIES}
)
