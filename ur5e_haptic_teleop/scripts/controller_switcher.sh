#!/bin/bash

# 这是一个Bash脚本，用于通过ROS服务切换控制器

# 打印提示信息，并等待ros_control的控制器管理器服务可用，最长等待5秒
echo "Waiting for /controller_manager/switch_controller service..."
rosservice wait /controller_manager/switch_controller 5

echo "Service found. Attempting to switch controllers..."

# 调用服务，启动速度控制器，并停止所有可能冲突的轨迹控制器
# start_controllers: 我们要启动的控制器列表。`joint_group_vel_controller`是UR驱动提供的速度控制器名称。
# stop_controllers: 我们要停止的控制器列表。`scaled_pos_joint_traj_controller`和`pos_joint_traj_controller`是UR驱动默认的轨迹控制器。
# strictness: 1表示严格模式，如果任何一个控制器启动/停止失败，则整个操作失败。
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
stop_controllers: ['scaled_pos_joint_traj_controller', 'pos_joint_traj_controller']
strictness: 1
start_asap: false
timeout: 0.0"

echo "Controller switch command sent. The script will now idle."

# 让脚本保持运行，防止launch文件认为它已退出并尝试重启它
sleep infinity