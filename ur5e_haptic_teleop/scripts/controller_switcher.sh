#!/bin/bash

# 这是一个Bash脚本，用于通过ROS服务切换控制器

echo "Waiting for /controller_manager/switch_controller service..."
rosservice wait /controller_manager/switch_controller 10

echo "Service found. Attempting to switch controllers..."

# 调用服务，启动我们标准化的速度控制器，并停止所有可能冲突的轨迹控制器
# start_controllers: 启动我们新定义的速度控制器
# stop_controllers:  停止您系统中实际在运行的那个轨迹控制器 (eff_joint_traj_controller)
# strictness: 2 表示“尽力而为”，即使某些控制器启动/停止失败，也会尝试继续
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_velocity_controller']
stop_controllers: ['eff_joint_traj_controller', 'scaled_pos_joint_traj_controller', 'pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.0"

echo "Controller switch command sent. The script will now idle."
echo "Please run 'rosrun controller_manager controller_manager list' in a new terminal to verify."

# 让脚本保持运行
sleep infinity