#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geomagic_control/DeviceButtonEvent.h>
#include <Eigen/Geometry>

// 引入MoveIt库用于逆运动学(IK)
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

class HapticTeleopController
{
public:
    HapticTeleopController() :
        nh_("~"),
        is_active_(false),
        has_received_haptic_pose_(false)
    {
        // 加载机器人模型，这是进行IK解算的必需步骤
        robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
        robot_model_ = robot_model_loader_->getModel();
        robot_state_.reset(new moveit::core::RobotState(robot_model_));
        robot_state_->setToDefaultValues();
        joint_model_group_ = robot_model_->getJointModelGroup("manipulator");

        // 获取关节名称
        joint_names_ = joint_model_group_->getVariableNames();

        nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
        nh_.param<std::string>("robot_eef_frame", robot_eef_frame_, "tool0");
        nh_.param<double>("linear_scale", linear_scale_, 1.0); // 位置模式下，缩放因子通常为1
        nh_.param<double>("angular_scale", angular_scale_, 1.0);
        nh_.param<double>("loop_rate", loop_rate_, 100.0);

        // 发布器改为发布轨迹消息
        traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 10);
        
        // 订阅器保持不变
        haptic_pose_sub_ = nh_.subscribe("/Geomagic/pose", 10, &HapticTeleopController::hapticPoseCallback, this);
        button_sub_ = nh_.subscribe("/Geomagic/button", 10, &HapticTeleopController::buttonCallback, this);
        joint_state_sub_ = nh_.subscribe("/joint_states", 1, &HapticTeleopController::jointStateCallback, this);

        ROS_INFO("Haptic Teleop Controller (Position Control Mode) initialized.");
        ROS_INFO("Now publishing to /eff_joint_traj_controller/command.");
    }

    void run()
    {
        ros::spin(); // 使用spin即可，所有逻辑都在回调中
    }

private:
    void transformMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& eigen_iso)
    {
        eigen_iso.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
        Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
        eigen_iso.linear() = q.toRotationMatrix();
    }
    
    // 存储机器人当前关节状态，用于IK解算的种子点
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
         robot_state_->setVariablePositions(msg->name, msg->position);
    }

    void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        latest_haptic_pose_msg_ = *msg;
        has_received_haptic_pose_ = true;

        if (is_active_)
        {
            calculateAndPublishTrajectory();
        }
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
            ROS_INFO("Teleoperation DEACTIVATED.");
        }
    }

    bool recordInitialPoses()
    {
        // 获取机器人初始位姿
        initial_robot_pose_ = robot_state_->getGlobalLinkTransform(robot_eef_frame_);

        // 获取触觉设备初始位姿
        if (!has_received_haptic_pose_) {
            ROS_WARN("Waiting for first haptic pose message...");
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/Geomagic/pose", ros::Duration(2.0));
            if (!has_received_haptic_pose_) {
                ROS_ERROR("Never received haptic pose message. Cannot activate.");
                return false;
            }
        }
        transformMsgToEigen(latest_haptic_pose_msg_.pose, initial_haptic_pose_in_haptic_frame_);
        initial_haptic_pose_in_haptic_frame_.translation() /= 1000.0; // mm to meters
        return true;
    }

    void calculateAndPublishTrajectory()
    {
        if (!has_received_haptic_pose_) return;

        Eigen::Isometry3d current_haptic_pose_in_haptic_frame;
        transformMsgToEigen(latest_haptic_pose_msg_.pose, current_haptic_pose_in_haptic_frame);
        current_haptic_pose_in_haptic_frame.translation() /= 1000.0; // mm to meters

        Eigen::Isometry3d haptic_delta = initial_haptic_pose_in_haptic_frame_.inverse() * current_haptic_pose_in_haptic_frame;
        
        Eigen::Isometry3d target_robot_pose = initial_robot_pose_ * haptic_delta;

        // 进行逆运动学解算
        std::vector<double> joint_values;
        bool found_ik = robot_state_->setFromIK(joint_model_group_, target_robot_pose, 0.1); // 0.1s timeout

        if (found_ik)
        {
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);

            // 创建并发布轨迹消息
            trajectory_msgs::JointTrajectory traj_msg;
            traj_msg.header.stamp = ros::Time::now();
            traj_msg.joint_names = joint_names_;
            
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = joint_values;
            point.time_from_start = ros::Duration(0.1); // 快速响应
            
            traj_msg.points.push_back(point);
            traj_pub_.publish(traj_msg);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "IK solution not found.");
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber joint_state_sub_;

    // MoveIt 相关对象
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<std::string> joint_names_;

    bool is_active_;
    bool has_received_haptic_pose_;
    geometry_msgs::PoseStamped latest_haptic_pose_msg_;
    Eigen::Isometry3d initial_robot_pose_;
    Eigen::Isometry3d initial_haptic_pose_in_haptic_frame_;
    std::string robot_base_frame_, robot_eef_frame_;
    double linear_scale_, angular_scale_, loop_rate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_teleop_node");
    ros::NodeHandle nh; // 需要一个公共句柄来等待参数
    // 等待 robot_description 参数加载
    while (!nh.hasParam("robot_description") && ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Waiting for robot_description on parameter server...");
        ros::Duration(0.5).sleep();
    }

    HapticTeleopController controller;
    controller.run();
    return 0;
}