#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geomagic_control/DeviceFeedback.h"
#include "visualization_msgs/Marker.h"
#include <cmath>

geometry_msgs::Pose current_pose;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virtual_sphere_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
    ros::Publisher force_pub = nh.advertise<geomagic_control::DeviceFeedback>("/Geomagic/force_feedback", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    double sphere_radius = 25.0;
    geometry_msgs::Point sphere_center;
    sphere_center.x = 0;
    sphere_center.y = 0;
    sphere_center.z = 0;

    ros::Rate loop_rate(1000); 

    while (ros::ok())
    {
        double dx = current_pose.position.x - sphere_center.x;
        double dy = current_pose.position.y - sphere_center.y;
        double dz = current_pose.position.z - sphere_center.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        geomagic_control::DeviceFeedback feedback_msg;
        if (distance < sphere_radius && distance > 0.001)
        {
            const double stiffness = 1.0; 
            double penetration = sphere_radius - distance;
            double force_magnitude = stiffness * penetration;
            
            feedback_msg.force.x = (dx / distance) * force_magnitude;
            feedback_msg.force.y = (dy / distance) * force_magnitude;
            feedback_msg.force.z = (dz / distance) * force_magnitude;
        }
        else
        {
            feedback_msg.force.x = 0;
            feedback_msg.force.y = 0;
            feedback_msg.force.z = 0;
        }

        // ===================== 最终修正处 =====================
        //
        //  在发布消息前，必须初始化lock列表，以防止驱动节点崩溃。
        //  我们不使用锁定功能，所以全部设为0（解锁状态）。
        //
        feedback_msg.lock.push_back(0);
        feedback_msg.lock.push_back(0);
        feedback_msg.lock.push_back(0);
        //
        // ======================================================

        force_pub.publish(feedback_msg);

        // 发布RVIZ中显示的球体Marker (这部分代码无需改动)
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "base";
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "haptic_base";
        sphere_marker.id = 0;
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.pose.position = sphere_center;
        sphere_marker.pose.orientation.w = 1.0;
        sphere_marker.scale.x = sphere_radius * 2.0;
        sphere_marker.scale.y = sphere_radius * 2.0;
        sphere_marker.scale.z = sphere_radius * 2.0;
        sphere_marker.color.a = 0.5; 
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 1.0;
        sphere_marker.color.b = 0.0;
        marker_pub.publish(sphere_marker);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}