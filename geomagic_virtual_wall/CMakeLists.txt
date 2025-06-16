cmake_minimum_required(VERSION 3.0.2)
project(geomagic_virtual_wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  geomagic_control
  visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wall_demo src/wall_demo.cpp)
target_link_libraries(wall_demo
  ${catkin_LIBRARIES}
)

# 添加可执行文件
# 将你的源文件(multi_object_demo.cpp)编译成一个名为 "multi_object_node" 的可执行文件
add_executable(multi_object_demo src/multi_object_demo.cpp)


# 链接可执行文件到Catkin库
target_link_libraries(multi_object_demo
  ${catkin_LIBRARIES}
)