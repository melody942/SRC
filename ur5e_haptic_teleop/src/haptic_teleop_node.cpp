#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geomagic_control/DeviceButtonEvent.h>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

// 定义控制模式的枚举
enum ControlMode
{
    INACTIVE,
    TRANSLATION,
    ROTATION
};

class HapticTeleopController
{
public:
    HapticTeleopController() :
        nh_("~"),
        control_mode_(INACTIVE),
        has_received_haptic_pose_(false),
        tf_listener_(tf_buffer_)
    {
        // 初始化机器人模型
        robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
        robot_model_ = robot_model_loader_->getModel();
        robot_state_.reset(new moveit::core::RobotState(robot_model_));
        robot_state_->setToDefaultValues();
        joint_model_group_ = robot_model_->getJointModelGroup("manipulator");
        joint_names_ = joint_model_group_->getVariableNames();

        // 加载参数
        nh_.param<std::string>("robot_eef_frame", robot_eef_frame_, "tool0");
        nh_.param<std::string>("haptic_base_frame", haptic_base_frame_, "haptic_base");
        nh_.param<double>("translation_scale_x", translation_scale_[0], 1.0);
        nh_.param<double>("translation_scale_y", translation_scale_[1], 1.0);
        nh_.param<double>("translation_scale_z", translation_scale_[2], 1.0);
        nh_.param<double>("rotation_scale", rotation_scale_, 1.0);
        nh_.param<double>("max_velocity", max_velocity_, 0.1);
        nh_.param<double>("max_acceleration", max_acceleration_, 0.1);

        // 初始化发布者和订阅者
        traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 10);
        haptic_pose_sub_ = nh_.subscribe("/Geomagic/pose", 10, &HapticTeleopController::hapticPoseCallback, this);
        button_sub_ = nh_.subscribe("/Geomagic/button", 10, &HapticTeleopController::buttonCallback, this);
        joint_state_sub_ = nh_.subscribe("/joint_states", 1, &HapticTeleopController::jointStateCallback, this);

        ROS_INFO("Haptic Teleop Controller initialized.");
        ROS_INFO("Hold GREY button for Position control. Hold WHITE button for Rotation control.");
    }

    void run() { ros::spin(); }

private:
    void transformMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& eigen_iso)
    {
        eigen_iso.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
        Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        eigen_iso.linear() = q.toRotationMatrix();
    }
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        robot_state_->setVariablePositions(msg->name, msg->position);
    }

    void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        // 直接使用原始消息，不做TF变换
        latest_haptic_pose_msg_ = *msg;
        has_received_haptic_pose_ = true;

        if (control_mode_ != INACTIVE)
        {
            calculateAndPublishTrajectory();
        }
    }

    void buttonCallback(const geomagic_control::DeviceButtonEvent& msg)
    {
        ControlMode new_mode = INACTIVE;
        if (msg.white_button == 1) {
            new_mode = ROTATION;
        } else if (msg.grey_button == 1) {
            new_mode = TRANSLATION;
        }

        if (new_mode != control_mode_) {
            control_mode_ = new_mode;
            if (control_mode_ != INACTIVE) {
                ROS_INFO("Teleoperation Mode Changed to: %s", 
                    (control_mode_ == TRANSLATION) ? "TRANSLATION" : "ROTATION");
                recordInitialPoses();
            } else {
                ROS_INFO("Teleoperation DEACTIVATED.");
            }
        }
    }

    void recordInitialPoses()
    {
        // 记录机器人初始位姿
        initial_robot_pose_ = robot_state_->getGlobalLinkTransform(robot_eef_frame_);
        
        // 记录触觉设备初始位姿
        if (!has_received_haptic_pose_) {
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/Geomagic/pose", ros::Duration(1.0));
            if(!has_received_haptic_pose_) {
                ROS_ERROR("Cannot record initial poses, no haptic message received.");
                control_mode_ = INACTIVE;
                return;
            }
        }
        transformMsgToEigen(latest_haptic_pose_msg_.pose, initial_haptic_pose_);
        ROS_INFO("Initial poses recorded for new control mode.");
    }

    void calculateAndPublishTrajectory()
    {
        if (!has_received_haptic_pose_) return;

        // 获取当前触觉设备位姿
        Eigen::Isometry3d current_haptic_pose;
        transformMsgToEigen(latest_haptic_pose_msg_.pose, current_haptic_pose);

        // 计算位姿变化
        Eigen::Isometry3d haptic_delta = initial_haptic_pose_.inverse() * current_haptic_pose;
        Eigen::Isometry3d target_robot_pose = initial_robot_pose_;

        if (control_mode_ == TRANSLATION)
        {
            // 应用缩放和平移
            Eigen::Vector3d translation = haptic_delta.translation();
            translation.x() *= translation_scale_[0];
            translation.y() *= translation_scale_[1];
            translation.z() *= translation_scale_[2];
            
            // 限制最大速度
            double translation_norm = translation.norm();
            if (translation_norm > max_velocity_) {
                translation *= max_velocity_ / translation_norm;
            }
            
            target_robot_pose.translation() += translation;
        }
        else if (control_mode_ == ROTATION)
        {
            // 提取旋转轴和角度
            Eigen::AngleAxisd rotation_delta(haptic_delta.rotation());
            double angle = rotation_delta.angle() * rotation_scale_;
            
            // 限制最大角速度
            if (angle > max_velocity_) {
                angle = max_velocity_;
            }
            
            Eigen::AngleAxisd scaled_rotation(angle, rotation_delta.axis());
            target_robot_pose.rotate(scaled_rotation);
        }

        // 求解逆运动学
        std::vector<double> joint_values;
        bool found_ik = robot_state_->setFromIK(joint_model_group_, target_robot_pose, 0.1);

        if (found_ik)
        {
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
            
            // 创建轨迹消息
            trajectory_msgs::JointTrajectory traj_msg;
            traj_msg.header.stamp = ros::Time::now();
            traj_msg.joint_names = joint_names_;
            
            // 添加轨迹点
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = joint_values;
            point.velocities.resize(joint_values.size(), 0.0);
            point.accelerations.resize(joint_values.size(), 0.0);
            point.time_from_start = ros::Duration(0.1);
            
            traj_msg.points.push_back(point);
            traj_pub_.publish(traj_msg);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "IK solution not found!");
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber joint_state_sub_;
    ControlMode control_mode_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<std::string> joint_names_;

    bool has_received_haptic_pose_;
    geometry_msgs::PoseStamped latest_haptic_pose_msg_;
    Eigen::Isometry3d initial_robot_pose_;
    Eigen::Isometry3d initial_haptic_pose_;
    
    std::string robot_eef_frame_;
    std::string haptic_base_frame_;
    std::array<double, 3> translation_scale_;
    double rotation_scale_;
    double max_velocity_;
    double max_acceleration_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_teleop_node");
    ros::NodeHandle nh;
    
    while (!nh.hasParam("robot_description") && ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Waiting for robot_description on parameter server...");
        ros::Duration(0.5).sleep();
    }
    
    HapticTeleopController controller;
    controller.run();
    return 0;
}