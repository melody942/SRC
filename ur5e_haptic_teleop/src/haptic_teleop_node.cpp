#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geomagic_control/DeviceButtonEvent.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

class HapticTeleopController
{
public:
    HapticTeleopController() :
        nh_("~"),
        tf_listener_(tf_buffer_),
        is_active_(false),
        has_received_haptic_pose_(false)
    {
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<std::string>("robot_eef_frame", robot_eef_frame_, "tool0");
        nh_.param<double>("linear_scale", linear_scale_, 0.5);
        nh_.param<double>("angular_scale", angular_scale_, 0.5);
        nh_.param<double>("p_gain_linear", p_gain_linear_, 2.0);
        nh_.param<double>("p_gain_angular", p_gain_angular_, 1.5);
        nh_.param<double>("loop_rate", loop_rate_, 100.0);

        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);
        haptic_pose_sub_ = nh_.subscribe("/Geomagic/pose", 10, &HapticTeleopController::hapticPoseCallback, this);
        button_sub_ = nh_.subscribe("/Geomagic/button", 10, &HapticTeleopController::buttonCallback, this);

        ROS_INFO("Haptic Teleop Controller initialized.");
        ROS_INFO("Press the grey button on the haptic device to start/stop teleoperation.");
    }

    void run()
    {
        ros::Rate rate(loop_rate_);
        while (ros::ok())
        {
            if (is_active_)
            {
                calculateAndPublishTwist();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // ===================== 核心修正区域 =====================
    // 提供两个版本的函数，一个处理Transform，一个处理Pose
    
    // 版本1: 处理来自TF的 geometry_msgs::Transform
    void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Isometry3d& eigen_iso)
    {
        eigen_iso.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);
        Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
        eigen_iso.linear() = q.toRotationMatrix();
    }
    
    // 版本2: 处理来自话题的 geometry_msgs::Pose
    void transformMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& eigen_iso)
    {
        eigen_iso.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
        Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        eigen_iso.linear() = q.toRotationMatrix();
    }
    // =======================================================

    void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        latest_haptic_pose_msg_ = *msg;
        has_received_haptic_pose_ = true;
    }

    void buttonCallback(const geomagic_control::DeviceButtonEvent& msg)
    {
        if (msg.grey_button == 1 && !is_active_)
        {
            if (recordInitialPoses()) {
                is_active_ = true;
                ROS_INFO("Teleoperation ACTIVATED.");
            } else {
                ROS_WARN("Failed to record initial poses. Teleoperation remains DEACTIVATED.");
            }
        }
        else if (msg.grey_button == 0 && is_active_)
        {
            is_active_ = false;
            publishZeroTwist();
            ROS_INFO("Teleoperation DEACTIVATED.");
        }
    }

    bool recordInitialPoses()
    {
        try
        {
            geometry_msgs::TransformStamped robot_tf = tf_buffer_.lookupTransform(robot_base_frame_, robot_eef_frame_, ros::Time(0), ros::Duration(1.0));
            transformMsgToEigen(robot_tf.transform, initial_robot_pose_); // 现在这一行可以正确编译了
            
            if (!has_received_haptic_pose_) {
                ROS_WARN("Waiting for first haptic pose message on /Geomagic/pose...");
                ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/Geomagic/pose", ros::Duration(2.0));
                if (!has_received_haptic_pose_) {
                    ROS_ERROR("Timeout waiting for haptic pose message. Is the driver running?");
                    return false;
                }
            }

            transformMsgToEigen(latest_haptic_pose_msg_.pose, initial_haptic_pose_in_haptic_frame_);
            initial_haptic_pose_in_haptic_frame_.translation() /= 1000.0;

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Could not get robot pose from TF. %s", ex.what());
            return false;
        }
    }

    void calculateAndPublishTwist()
    {
        if (!has_received_haptic_pose_) return;

        try
        {
            Eigen::Isometry3d current_robot_pose;
            geometry_msgs::TransformStamped current_robot_tf_msg = tf_buffer_.lookupTransform(robot_base_frame_, robot_eef_frame_, ros::Time(0));
            transformMsgToEigen(current_robot_tf_msg.transform, current_robot_pose); // 现在这一行也可以正确编译了

            Eigen::Isometry3d current_haptic_pose_in_haptic_frame;
            transformMsgToEigen(latest_haptic_pose_msg_.pose, current_haptic_pose_in_haptic_frame);
            current_haptic_pose_in_haptic_frame.translation() /= 1000.0;

            Eigen::Isometry3d haptic_delta = initial_haptic_pose_in_haptic_frame_.inverse() * current_haptic_pose_in_haptic_frame;
            
            Eigen::Vector3d scaled_translation = haptic_delta.translation() * linear_scale_;
            Eigen::AngleAxisd angle_axis(haptic_delta.rotation());
            Eigen::AngleAxisd scaled_rotation(angle_axis.angle() * angular_scale_, angle_axis.axis());
            
            Eigen::Isometry3d scaled_haptic_delta = Eigen::Isometry3d::Identity();
            scaled_haptic_delta.translate(scaled_translation);
            scaled_haptic_delta.rotate(scaled_rotation);

            Eigen::Isometry3d target_robot_pose = initial_robot_pose_ * scaled_haptic_delta;

            Eigen::Vector3d pos_error = target_robot_pose.translation() - current_robot_pose.translation();
            Eigen::Vector3d linear_velocity = p_gain_linear_ * pos_error;

            Eigen::Quaterniond q_error = Eigen::Quaterniond(target_robot_pose.rotation()) * Eigen::Quaterniond(current_robot_pose.rotation()).inverse();
            Eigen::AngleAxisd error_angle_axis(q_error);
            Eigen::Vector3d angular_velocity = p_gain_angular_ * error_angle_axis.axis() * error_angle_axis.angle();

            geometry_msgs::TwistStamped twist_msg;
            twist_msg.header.stamp = ros::Time::now();
            twist_msg.header.frame_id = robot_base_frame_;
            twist_msg.twist.linear.x = linear_velocity.x();
            twist_msg.twist.linear.y = linear_velocity.y();
            twist_msg.twist.linear.z = linear_velocity.z();
            twist_msg.twist.angular.x = angular_velocity.x();
            twist_msg.twist.angular.y = angular_velocity.y();
            twist_msg.twist.angular.z = angular_velocity.z();
            twist_pub_.publish(twist_msg);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("Could not get robot pose from TF in control loop. %s", ex.what());
            publishZeroTwist();
        }
    }

    void publishZeroTwist()
    {
        geometry_msgs::TwistStamped zero_twist;
        zero_twist.header.stamp = ros::Time::now();
        zero_twist.header.frame_id = robot_base_frame_;
        twist_pub_.publish(zero_twist);
    }

    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber button_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool is_active_;
    bool has_received_haptic_pose_;
    geometry_msgs::PoseStamped latest_haptic_pose_msg_;
    Eigen::Isometry3d initial_robot_pose_;
    Eigen::Isometry3d initial_haptic_pose_in_haptic_frame_;
    std::string robot_base_frame_, robot_eef_frame_;
    double linear_scale_, angular_scale_, p_gain_linear_, p_gain_angular_, loop_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_teleop_node");
    HapticTeleopController controller;
    controller.run();
    return 0;
}