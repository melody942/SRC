#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <geomagic_control/DeviceFeedback.h>
#include <vector>
#include <cmath>

// 全局变量
ros::Publisher g_pub_force;
ros::Publisher g_pub_marker;
geometry_msgs::Pose g_current_pose; // 存储最新的设备位姿 (单位: mm)

// 定义虚拟物体结构体
struct VirtualObject
{
    int id;
    int type;
    // 参数全部使用毫米(mm)单位
    geometry_msgs::Point position_mm; 
    geometry_msgs::Vector3 scale_mm;   
    std_msgs::ColorRGBA color;
    double stiffness;
};

// 全局虚拟物体列表
std::vector<VirtualObject> g_virtual_objects;

// 回调函数
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_current_pose = msg->pose;
}

// 初始化函数
void setupVirtualObjects()
{
    g_virtual_objects.clear(); // 清空列表以防重复添加

    // --- 修正1：调整所有物体的位置到可触及范围内 ---

    // 物体1：垂直的墙 (蓝色)，位置微调
    VirtualObject wall;
    wall.id = 0;
    wall.type = visualization_msgs::Marker::CUBE;
    wall.position_mm.x = 70.0;     // X=70mm (比之前近一点)
    wall.position_mm.y = 0.0;
    wall.position_mm.z = 40.0;
    wall.scale_mm.x = 10.0;        // 10mm 厚
    wall.scale_mm.y = 100.0;       // 100mm 宽
    wall.scale_mm.z = 100.0;       // 100mm 高
    wall.color.r = 0.0; wall.color.g = 0.0; wall.color.b = 1.0; wall.color.a = 0.6;
    wall.stiffness = 0.25;
    g_virtual_objects.push_back(wall);

    // 物体2：一个悬浮的球 (红色)，移到右前方
    VirtualObject sphere;
    sphere.id = 1;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.position_mm.x = 40.0;  // X=40mm
    sphere.position_mm.y = -40.0; // Y=-40mm (右边)
    sphere.position_mm.z = 50.0;  // Z=50mm (上方)
    sphere.scale_mm.x = 40.0;     // 直径40mm的球
    sphere.scale_mm.y = 40.0;
    sphere.scale_mm.z = 40.0;
    sphere.color.r = 1.0; sphere.color.g = 0.0; sphere.color.b = 0.0; sphere.color.a = 0.6;
    sphere.stiffness = 0.3;
    g_virtual_objects.push_back(sphere);

    // 物体3：一个躺着的路障 (绿色)，移到左前方地面
    VirtualObject block;
    block.id = 2;
    block.type = visualization_msgs::Marker::CUBE;
    block.position_mm.x = 50.0;    // X=50mm
    block.position_mm.y = 30.0;    // Y=30mm (左边)
    block.position_mm.z = 10.0;    // Z=10mm (贴近地面)
    block.scale_mm.x = 60.0;       // 60mm长
    block.scale_mm.y = 20.0;       // 20mm宽
    block.scale_mm.z = 20.0;       // 20mm高
    block.color.r = 0.0; block.color.g = 1.0; block.color.b = 0.0; block.color.a = 0.6;
    block.stiffness = 0.15;
    g_virtual_objects.push_back(block);
}

// 主循环
int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_object_demo_node");
    ros::NodeHandle nh;

    setupVirtualObjects();

    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>("/Geomagic/pose", 3, poseCallback);
    g_pub_force = nh.advertise<geomagic_control::DeviceFeedback>("/Geomagic/force_feedback", 10);
    g_pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate rate(500); // 建议使用更高的频率以获得更稳定的力反馈

    while(ros::ok())
    {
        ros::spinOnce();

        const geometry_msgs::Point& stylus_pos_mm = g_current_pose.position;
        geometry_msgs::Vector3 total_force;

        for (const auto& object : g_virtual_objects)
        {
            geometry_msgs::Vector3 object_force;

            // --- 核心物理逻辑 (所有计算均在mm单位下) ---
            if (object.type == visualization_msgs::Marker::CUBE)
            {
                // ... 立方体的逻辑是正确的，保持不变 ...
                double half_x = object.scale_mm.x / 2.0;
                double half_y = object.scale_mm.y / 2.0;
                double half_z = object.scale_mm.z / 2.0;

                if (fabs(stylus_pos_mm.x - object.position_mm.x) < half_x &&
                    fabs(stylus_pos_mm.y - object.position_mm.y) < half_y &&
                    fabs(stylus_pos_mm.z - object.position_mm.z) < half_z)
                {
                    double dx = half_x - fabs(stylus_pos_mm.x - object.position_mm.x);
                    double dy = half_y - fabs(stylus_pos_mm.y - object.position_mm.y);
                    double dz = half_z - fabs(stylus_pos_mm.z - object.position_mm.z);
                    double penetration = std::min({dx, dy, dz});
                    
                    geometry_msgs::Vector3 normal;
                    if (penetration == dx) normal.x = -(stylus_pos_mm.x - object.position_mm.x > 0 ? 1 : -1);
                    else if (penetration == dy) normal.y = -(stylus_pos_mm.y - object.position_mm.y > 0 ? 1 : -1);
                    else normal.z = -(stylus_pos_mm.z - object.position_mm.z > 0 ? 1 : -1);
                    
                    object_force.x = -object.stiffness * penetration * normal.x;
                    object_force.y = -object.stiffness * penetration * normal.y;
                    object_force.z = -object.stiffness * penetration * normal.z;
                }
            }
            else if (object.type == visualization_msgs::Marker::SPHERE)
            {
                double radius = object.scale_mm.x / 2.0;
                double dist_x = stylus_pos_mm.x - object.position_mm.x;
                double dist_y = stylus_pos_mm.y - object.position_mm.y;
                double dist_z = stylus_pos_mm.z - object.position_mm.z;
                double distance = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

                if (distance < radius && distance > 1e-6)
                {
                    double penetration = radius - distance;
                    // --- 修正3：去掉了这里多余的负号 ---
                    // 力的方向(法线)是从球心指向触控笔，大小是(硬度*穿透深度)
                    object_force.x = object.stiffness * penetration * (dist_x / distance);
                    object_force.y = object.stiffness * penetration * (dist_y / distance);
                    object_force.z = object.stiffness * penetration * (dist_z / distance);
                }
            }
            
            total_force.x += object_force.x;
            total_force.y += object_force.y;
            total_force.z += object_force.z;

            // --- RVIZ 可视化逻辑 ---
            visualization_msgs::Marker marker;
            marker.header.frame_id = "base";
            marker.header.stamp = ros::Time::now();
            marker.ns = "virtual_objects";
            marker.id = object.id;
            marker.type = object.type;
            marker.action = visualization_msgs::Marker::ADD;

            // --- 修正2：只有X轴取负，Y和Z轴保持正向 ---
            marker.pose.position.x = -object.position_mm.x * 0.001;
            marker.pose.position.y =  object.position_mm.y * 0.001; // 不再取负
            marker.pose.position.z =  object.position_mm.z * 0.001; // 不再取负
            
            marker.pose.orientation.w = 1.0;

            marker.scale.x = object.scale_mm.x * 0.001;
            marker.scale.y = object.scale_mm.y * 0.001;
            marker.scale.z = object.scale_mm.z * 0.001;
            
            marker.color = object.color;
            marker.lifetime = ros::Duration();
            g_pub_marker.publish(marker);
        }

        // --- 发布力反馈 ---
        geomagic_control::DeviceFeedback feedback_msg;
        feedback_msg.force = total_force;
        feedback_msg.lock = {0, 0, 0};
        g_pub_force.publish(feedback_msg);

        rate.sleep();
    }

    return 0;
}