# =======================================================
#   haptic_from_scratch/CMakeLists.txt 的最终无误版
# =======================================================
cmake_minimum_required(VERSION 3.0.2)
project(haptic_from_scratch)

# --- 第一步：查找所有ROS依赖包 ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
)

# --- 第二步：全局声明所有头文件和库文件的搜索路径 ---
# 这是最标准、最稳妥的做法，确保所有目标都能找到所需文件
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/HD
)
link_directories(
  /usr/lib/x86_64-linux-gnu
)

# --- 第三步：Catkin包声明 ---
catkin_package()

# --- 第四步：定义并链接我们的节点 ---
add_executable(haptic_world_node src/haptic_world_node.cpp)

# 确保在编译此节点前，所有依赖的消息都已生成
# 这里使用了正确的变量名
add_dependencies(haptic_world_node ${catkin_EXPORTED_TARGETS})

# 链接所有需要的库 (HDAPI库 和 ROS库)
target_link_libraries(haptic_world_node
  HDU
  HD
  ${catkin_LIBRARIES}
)
