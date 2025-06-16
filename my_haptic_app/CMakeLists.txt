# ==================================================
#         CMakeLists.txt 的最终正确版
# ==================================================
cmake_minimum_required(VERSION 3.0.2)
project(my_haptic_app)

# --- 第一步：查找所有需要的依赖包 ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geomagic_control
  visualization_msgs
)

# --- 第二步：声明所有头文件和库文件的搜索路径 ---
# 这个部分放在最前面，作用于整个项目，避免任何作用域问题
include_directories(
  # ROS的头文件路径
  ${catkin_INCLUDE_DIRS}
  # OpenHaptics的头文件路径
  /usr/include/QH
)

link_directories(
  # OpenHaptics的库文件路径
  /usr/lib/x86_64-linux-gnu
)

# --- 第三步：Catkin包声明 ---
catkin_package()

# --- 第四步：定义并链接您的可执行文件 ---

# 目标A: ROS交互节点
add_executable(virtual_sphere_node src/virtual_sphere_node.cpp)
add_dependencies(virtual_sphere_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(virtual_sphere_node
  ${catkin_LIBRARIES}
)

# 目标B: 独立显示节点 (可选)
add_executable(SolidSphereNode src/SolidSphere.cpp)
target_link_libraries(SolidSphereNode
  QHGLUTWrapper QH HL HDU HD
  GL GLU glut z
)