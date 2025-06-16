#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>



// 定义一个类来封装遥操作逻辑
class HapticTeleopController
{
public:
    HapticTeleopController() :
        nh_("~"), // 使用私有节点句柄，方便参数管理
        tf_listener_(tf_buffer_),
        is_active_(false),
        is_first_activation_(true)
    {
        // 从参数服务器获取配置
        // 这些参数应在launch文件中设置
        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<std::string>("robot_eef_frame", robot_eef_frame_, "tool0");
        nh_.param<std::string>("haptic_frame", haptic_frame_, "omni1_gripper");
        nh_.param<double>("linear_scale", linear_scale_, 0.5);
        nh_.param<double>("angular_scale", angular_scale_, 0.5);
        nh_.param<double>("p_gain_linear", p_gain_linear_, 2.0);
        nh_.param<double>("p_gain_angular", p_gain_angular_, 1.5);
        nh_.param<double>("loop_rate", loop_rate_, 100.0);

        // 初始化ROS发布者和订阅者
        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 10);
        haptic_pose_sub_ = nh_.subscribe("/omni/pose", 10, &HapticTeleopController::hapticPoseCallback, this);
        button_sub_ = nh_.subscribe("/omni/button_event", 10, &HapticTeleopController::buttonCallback, this);

        ROS_INFO("Haptic Teleop Controller initialized.");
        ROS_INFO("Robot Base Frame: %s", robot_base_frame_.c_str());
        ROS_INFO("Robot EEF Frame: %s", robot_eef_frame_.c_str());
        ROS_INFO("Press the grey button on the haptic device to start/stop teleoperation.");
    }

    // 主循环
    void run()
    {
        ros::Rate rate(loop_rate_);
        while (ros::ok())
        {
            if (is_active_)
            {
                // 计算并发布速度指令
                calculateAndPublishTwist();
            }
            else
            {
                // 如果遥操作未激活，确保机器人停止
                publishZeroTwist();
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // 按钮事件回调函数
    void buttonCallback(const omni_msgs::OmniButtonEvent::ConstPtr& msg)
    {
        // 我们使用灰色按钮作为“离合器”
        if (msg->grey_button == 1 &&!is_active_)
        {
            is_active_ = true;
            if (is_first_activation_)
            {
                // 首次激活时，记录初始位姿
                if (recordInitialPoses())
                {
                    is_first_activation_ = false;
                    ROS_INFO("Teleoperation ACTIVATED. Initial poses recorded.");
                }
                else
                {
                    // 如果无法获取初始位姿，则保持非激活状态
                    is_active_ = false;
                    ROS_WARN("Failed to record initial poses. Teleoperation remains DEACTIVATED.");
                }
            }
            else
            {
                 // 如果不是首次激活，只需重新记录初始位姿即可
                 if(recordInitialPoses()){
                    ROS_INFO("Teleoperation RE-ACTIVATED.");
                 } else {
                    is_active_ = false;
                    ROS_WARN("Failed to re-record initial poses. Teleoperation remains DEACTIVATED.");
                 }
            }
        }
        else if (msg->grey_button == 0 && is_active_)
        {
            is_active_ = false;
            ROS_INFO("Teleoperation DEACTIVATED.");
        }
    }

    // 触觉设备位姿回调函数
    void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 简单地存储最新的位姿，计算在主循环中进行
        latest_haptic_pose_ = *msg;
    }

    // 记录初始位姿的函数
    bool recordInitialPoses()
    {
        try
        {
            // 记录机器人末端的初始位姿
            geometry_msgs::TransformStamped robot_tf = tf_buffer_.lookupTransform(robot_base_frame_, robot_eef_frame_, ros::Time(0), ros::Duration(1.0));
            transformMsgToEigen(robot_tf.transform, initial_robot_pose_);

            // 记录触觉设备的初始位姿
            // 注意：触觉设备的/omni/pose通常是相对于其自身基座的，这里我们假设它与机器人基座的TF已经发布
            // 如果没有，我们需要获取最新的haptic pose并假设其参考系是固定的
            geometry_msgs::TransformStamped haptic_tf = tf_buffer_.lookupTransform(robot_base_frame_, haptic_frame_, ros::Time(0), ros::Duration(1.0));
            transformMsgToEigen(haptic_tf.transform, initial_haptic_pose_);

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("TF Exception in recordInitialPoses: %s", ex.what());
            return false;
        }
    }

    // 计算并发布Twist指令
    void calculateAndPublishTwist()
    {
        try
        {
            // 1. 获取当前机器人和触觉设备的位姿
            geometry_msgs::TransformStamped current_robot_tf_msg = tf_buffer_.lookupTransform(robot_base_frame_, robot_eef_frame_, ros::Time(0));
            Eigen::Isometry3d current_robot_pose;
            transformMsgToEigen(current_robot_tf_msg.transform, current_robot_pose);

            geometry_msgs::TransformStamped current_haptic_tf_msg = tf_buffer_.lookupTransform(robot_base_frame_, haptic_frame_, ros::Time(0));
            Eigen::Isometry3d current_haptic_pose;
            transformMsgToEigen(current_haptic_tf_msg.transform, current_haptic_pose);


            // 2. 计算触觉设备的相对运动 (delta)
            Eigen::Isometry3d haptic_delta = initial_haptic_pose_.inverse() * current_haptic_pose;

            // 3. 应用缩放因子到相对运动
            Eigen::Vector3d scaled_translation = haptic_delta.translation() * linear_scale_;
            Eigen::AngleAxisd angle_axis(haptic_delta.rotation());
            double scaled_angle = angle_axis.angle() * angular_scale_;
            Eigen::AngleAxisd scaled_rotation(scaled_angle, angle_axis.axis());

            Eigen::Isometry3d scaled_haptic_delta = Eigen::Isometry3d::Identity();
            scaled_haptic_delta.translate(scaled_translation);
            scaled_haptic_delta.rotate(scaled_rotation);

            // 4. 计算目标机器人位姿
            Eigen::Isometry3d target_robot_pose = initial_robot_pose_ * scaled_haptic_delta;

            // 5. 将位姿误差转换为速度指令 (P控制器)
            // 平移误差
            Eigen::Vector3d pos_error = target_robot_pose.translation() - current_robot_pose.translation();
            Eigen::Vector3d linear_velocity = p_gain_linear_ * pos_error;

            // 旋转误差
            Eigen::Quaterniond q_target(target_robot_pose.rotation());
            Eigen::Quaterniond q_current(current_robot_pose.rotation());
            Eigen::Quaterniond q_error = q_target * q_current.inverse();
            Eigen::AngleAxisd error_angle_axis(q_error);
            Eigen::Vector3d angular_velocity = p_gain_angular_ * error_angle_axis.axis() * error_angle_axis.angle();

            // 6. 构建并发布TwistStamped消息
            geometry_msgs::TwistStamped twist_msg;
            twist_msg.header.stamp = ros::Time::now();
            twist_msg.header.frame_id = robot_base_frame_; // 指令在基坐标系下
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
            ROS_ERROR("TF Exception in control loop: %s", ex.what());
            publishZeroTwist(); // 如果TF查询失败，发送停止指令
        }
    }
    // 手动将 geometry_msgs::Transform 转换为 Eigen::Isometry3d
    void transformMsgToEigen(const geometry_msgs::Transform& msg, Eigen::Isometry3d& eigen_iso)
{
    // 从消息中提取平移向量
    eigen_iso.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);

    // 从消息中提取旋转四元数
    Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);

    // 将四元数转换为旋转矩阵并赋值
    eigen_iso.linear() = q.toRotationMatrix();
}
    // 发布零速度指令以停止机器人
    void publishZeroTwist()
    {
        geometry_msgs::TwistStamped zero_twist;
        zero_twist.header.stamp = ros::Time::now();
        zero_twist.header.frame_id = robot_base_frame_;
        // 所有速度分量默认为0
        twist_pub_.publish(zero_twist);
    }

    // ROS成员变量
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber button_sub_;

    // TF成员变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 控制逻辑状态变量
    bool is_active_;
    bool is_first_activation_;
    geometry_msgs::PoseStamped latest_haptic_pose_;
    Eigen::Isometry3d initial_robot_pose_;
    Eigen::Isometry3d initial_haptic_pose_;

    // 配置参数
    std::string robot_base_frame_;
    std::string robot_eef_frame_;
    std::string haptic_frame_;
    double linear_scale_;
    double angular_scale_;
    double p_gain_linear_;
    double p_gain_angular_;
    double loop_rate_;
};

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_teleop_node");
    HapticTeleopController controller;
    controller.run();
    return 0;
}