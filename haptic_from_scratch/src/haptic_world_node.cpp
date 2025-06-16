#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

struct HapticState
{
    hduVector3Dd position_mm; // 设备位置，单位是毫米
    hduVector3Dd force;
    HHD haptic_device;
};

HDCallbackCode HDCALLBACK haptic_callback(void *data)
{
    HapticState *state = static_cast<HapticState*>(data);
    hdBeginFrame(state->haptic_device);
    hdGetDoublev(HD_CURRENT_POSITION, state->position_mm);

    // --- 物理计算依然在毫米(mm)单位下进行 ---
    hduVector3Dd sphere_center_mm(0, 0, 0);
    HDdouble sphere_radius_mm = 25.0; // 物理半径 25mm

    hduVector3Dd vector_to_center = sphere_center_mm - state->position_mm;
    HDdouble distance_to_center = hduVecMagnitude(vector_to_center);

    if (distance_to_center < sphere_radius_mm)
    {
        const HDdouble stiffness = 0.5;
        HDdouble penetration_depth = sphere_radius_mm - distance_to_center;
        
        hduVecNormalize(vector_to_center, vector_to_center);
        hduVecScale(state->force, vector_to_center, penetration_depth * stiffness);
    }
    else
    {
        state->force.set(0,0,0);
    }

    hdSetDoublev(HD_CURRENT_FORCE, state->force);
    hdEndFrame(state->haptic_device);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Haptic callback error");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "haptic_world_node");
    ros::NodeHandle nh;

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(hdGetError())) {
        ROS_ERROR("Failed to initialize haptic device.");
        return -1;
    }
    hdEnable(HD_FORCE_OUTPUT);
    
    HapticState state;
    state.haptic_device = hHD;

    hdStartScheduler();
    HDCallbackCode h_callback = hdScheduleAsynchronous(haptic_callback, &state, HD_MAX_SCHEDULER_PRIORITY);

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("haptic_pose", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    static tf2_ros::TransformBroadcaster br;

    ros::Rate loop_rate(100);

    ROS_INFO("Haptic world node started. Ready to visualize in RVIZ.");

    while(ros::ok())
    {
        // 从力反馈状态中获取毫米单位的位置
        hduVector3Dd current_position_mm = state.position_mm;

        // ===================== 单位转换核心 (mm -> m) =====================
        // 将所有要发布到ROS的数据除以1000.0，转换为米
        double pos_x_m = current_position_mm[0] / 1000.0;
        double pos_y_m = current_position_mm[1] / 1000.0;
        double pos_z_m = current_position_mm[2] / 1000.0;
        
        double sphere_radius_m = 25.0 / 1000.0; // 可视化球体的半径，单位为米
        // =================================================================

        // --- 发布探针位姿话题 (使用米单位) ---
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "haptic_world";
        pose_msg.pose.position.x = pos_x_m;
        pose_msg.pose.position.y = pos_y_m;
        pose_msg.pose.position.z = pos_z_m;
        pose_msg.pose.orientation.w = 1.0;
        pose_pub.publish(pose_msg);

        // --- 发布TF变换 (使用米单位) ---
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "haptic_world";
        transformStamped.child_frame_id = "stylus";
        transformStamped.transform.translation.x = pos_x_m;
        transformStamped.transform.translation.y = pos_y_m;
        transformStamped.transform.translation.z = pos_z_m;
        transformStamped.transform.rotation.w = 1.0;
        br.sendTransform(transformStamped);

        // --- 发布虚拟球的可视化Marker (使用米单位) ---
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "haptic_world";
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "haptic_world";
        sphere_marker.id = 0;
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.pose.position.x = 0;
        sphere_marker.pose.position.y = 0;
        sphere_marker.pose.position.z = 0;
        sphere_marker.pose.orientation.w = 1.0;
        // Marker的scale是直径，所以是半径的2倍
        sphere_marker.scale.x = sphere_radius_m * 2.0;
        sphere_marker.scale.y = sphere_radius_m * 2.0;
        sphere_marker.scale.z = sphere_radius_m * 2.0;
        sphere_marker.color.a = 0.5;
        sphere_marker.color.r = 1.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 0.5;
        marker_pub.publish(sphere_marker);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Shutting down haptic world node.");
    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
}