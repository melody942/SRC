#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geomagic_control/DeviceFeedback.h"
#include "visualization_msgs/Marker.h"
#include <vector>
// 需要注意的是，在RVIZ中的单位默认为m，而实际环境中的单位默认为mm，因此需要有一个单位转换！ 本代码成功完全实现生成一堵墙 并且位置在RVIZ中可见
// 另外，墙体位置中的负号是必须的，否则会与看得相反（当然也可能是touch是以对面那个角度来看的）
// 全局变量，用于存储最新的设备位姿和发布器
ros::Publisher g_pub_force;
ros::Publisher g_pub_marker;
geometry_msgs::PoseStamped g_current_pose;

// 墙体参数
const double WALL_POSITION_X = 50; // 墙在X轴上的位置 (单位: mm)
const double STIFFNESS = 0.25; // 墙的“硬度”，这是一个需要微调的参数。值越大，墙感觉越硬。

// 设备位姿的回调函数
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_current_pose = *msg;
}

// 发布墙体Marker的函数，用于在RViz中显示
void publishWallMarker()
{
    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = "base"; // 确保与设备使用相同的坐标系
    wall_marker.header.stamp = ros::Time::now();
    wall_marker.ns = "virtual_wall";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::Marker::CUBE; // 使用立方体来代表墙
    wall_marker.action = visualization_msgs::Marker::ADD;

    // 设置墙体的位置
    wall_marker.pose.position.x = -WALL_POSITION_X*0.001; // [代码保持不变]
    wall_marker.pose.position.y = 0;
    wall_marker.pose.position.z = 0;
    double theta = 45.0 * M_PI / 180.0; // 转换为弧度

    wall_marker.pose.orientation.x = 0.0;
    wall_marker.pose.orientation.y = 0.0;
    wall_marker.pose.orientation.z = sin(theta / 2.0);
    wall_marker.pose.orientation.w = cos(theta / 2.0);

    // 设置墙体的大小
    wall_marker.scale.x = 0.0100;   // 墙的厚度
    wall_marker.scale.y = 0.5000; // 墙的宽度
    wall_marker.scale.z = 0.35000; // 墙的高度

    // 设置墙体的颜色 (半透明蓝色)
    wall_marker.color.r = 0.0f;
    wall_marker.color.g = 0.0f;
    wall_marker.color.b = 1.0f;
    wall_marker.color.a = 0.5; // Alpha值，0.5代表半透明

    wall_marker.lifetime = ros::Duration(); // 永久显示

    g_pub_marker.publish(wall_marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wall_demo_node");
    ros::NodeHandle nh;

    // 订阅设备位姿
    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>("/Geomagic/pose", 3, poseCallback);

    // 发布力反馈指令
    g_pub_force = nh.advertise<geomagic_control::DeviceFeedback>("/Geomagic/force_feedback", 10);
    
    // 发布RViz Marker
    g_pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Rate rate(100); // 设置循环频率为100Hz

    // --- 【修改部分开始】 ---
    // 定义与Marker一致的旋转角度
    const double wall_rotation_angle = 45.0 * M_PI / 180.0; 
    
    // 计算旋转后墙体的法线向量，用于物理计算
    const double wall_normal_x = cos(wall_rotation_angle);
    const double wall_normal_y = sin(wall_rotation_angle);
    // --- 【修改部分结束】 ---

    while(ros::ok())
    {
        ros::spinOnce(); // 处理回调函数

        geomagic_control::DeviceFeedback feedback_msg;
        double penetration_depth = 0.0;

        // --- 【核心物理逻辑 - 已修改】 ---
        // 计算触控笔位置在墙体法线方向上的投影距离
        double projected_distance = g_current_pose.pose.position.x * wall_normal_x + g_current_pose.pose.position.y * wall_normal_y;

        // 检查是否穿透了旋转后的墙面
        if (projected_distance > WALL_POSITION_X)
        {
            // 计算垂直于墙面的穿透深度
            penetration_depth = projected_distance - WALL_POSITION_X;

            // 根据胡克定律计算反作用力大小
            double force_magnitude = STIFFNESS * penetration_depth;
            
            // 将力沿着与法线相反的方向分解到X和Y轴
            feedback_msg.force.x = -force_magnitude * wall_normal_x;
            feedback_msg.force.y = -force_magnitude * wall_normal_y;
        }
        else
        {
            // 如果没有穿透，则力为0
            feedback_msg.force.x = 0.0;
            feedback_msg.force.y = 0.0; // 确保Y轴力也为0
        }

        feedback_msg.force.z = 0.0;
        
        // 确保所有轴都已解锁，以便施加力反馈
        std::vector<uint8_t> lockv(3, 0);
        feedback_msg.lock = lockv;
        
        // 发布力反馈消息
        g_pub_force.publish(feedback_msg);

        // 发布墙体Marker用于RViz显示
        publishWallMarker();

        rate.sleep();
    }

    return 0;
}