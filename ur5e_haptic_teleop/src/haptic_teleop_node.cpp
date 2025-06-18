#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geomagic_control/DeviceButtonEvent.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <cmath> // 用于 M_PI

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

class HapticTeleopController
{
public:
    HapticTeleopController(ros::NodeHandle& nh);
    void run();

private:
    // 函数声明
    void transformMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& eigen_iso);
    bool recordInitialPoses();
    void calculateAndPublishTrajectory();
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void buttonCallback(const geomagic_control::DeviceButtonEvent& msg);

    // 成员变量
    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber joint_state_sub_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<std::string> joint_names_;
    bool is_active_;
    bool has_received_haptic_pose_;
    bool has_received_joint_state_;
    geometry_msgs::PoseStamped latest_haptic_pose_msg_;
    Eigen::Isometry3d last_robot_pose_;
    Eigen::Isometry3d last_haptic_pose_in_haptic_frame_;
    std::string robot_base_frame_, robot_eef_frame_;
    double linear_scale_, angular_scale_, loop_rate_;
};

// 构造函数实现
HapticTeleopController::HapticTeleopController(ros::NodeHandle& nh) :
    nh_("~"),
    is_active_(false),
    has_received_haptic_pose_(false),
    has_received_joint_state_(false)
{
    ROS_INFO("Setting robot to initial pose by directly publishing to trajectory controller...");

    joint_names_ = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    traj_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 10);
    ros::Duration(1.0).sleep();

    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.joint_names = joint_names_;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = {
        207.0 * M_PI / 180.0, 
        -143.0 * M_PI / 180.0,
        52.0 * M_PI / 180.0,  
        -60.0 * M_PI / 180.0, 
        125.0 * M_PI / 180.0, 
        0.0 * M_PI / 180.0    
    };
    
    double move_duration = 5.0; 
    point.time_from_start = ros::Duration(move_duration);
    traj_msg.points.push_back(point);

    ROS_INFO("Publishing initial pose trajectory... Robot will move for %.1f seconds.", move_duration);
    traj_pub_.publish(traj_msg);

    ros::Duration(move_duration).sleep();
    ROS_INFO("Initial movement finished.");

    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();
    robot_state_.reset(new moveit::core::RobotState(robot_model_));
    robot_state_->setToDefaultValues();
    joint_model_group_ = robot_model_->getJointModelGroup("manipulator");

    linear_scale_ = 3; 
    angular_scale_ = 1.0; 
    nh_.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");
    nh_.param<std::string>("robot_eef_frame", robot_eef_frame_, "tool0");
    nh_.param<double>("loop_rate", loop_rate_, 100.0);
    
    haptic_pose_sub_ = nh_.subscribe("/Geomagic/pose", 10, &HapticTeleopController::hapticPoseCallback, this);
    button_sub_ = nh_.subscribe("/Geomagic/button", 10, &HapticTeleopController::buttonCallback, this);
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &HapticTeleopController::jointStateCallback, this);

    ROS_INFO("Haptic Teleop Controller (Incremental Mode) initialized.");
    ROS_INFO("Now publishing to /eff_joint_traj_controller/command.");
    
    ROS_INFO("Waiting for joint states for teleoperation...");
    ros::Rate r(10);
    while (!has_received_joint_state_ && ros::ok()) {
        r.sleep();
        ros::spinOnce(); 
    }
    ROS_INFO("Joint states received. Controller ready for teleoperation.");
}

// run函数实现
void HapticTeleopController::run()
{
    ros::spin();
}

// 以下是所有其他私有函数的实现（每个函数只出现一次）

void HapticTeleopController::transformMsgToEigen(const geometry_msgs::Pose& msg, Eigen::Isometry3d& eigen_iso)
{
    eigen_iso.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    q.normalize();
    eigen_iso.linear() = q.toRotationMatrix();
}

void HapticTeleopController::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::vector<double> positions;
    for (const auto& name : joint_names_) {
        auto it = std::find(msg->name.begin(), msg->name.end(), name);
        if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            positions.push_back(msg->position[idx]);
        } else {
            // 如果没找到，可以推入一个默认值或者保持上一个值，这里推入0.0
            positions.push_back(0.0);
        }
    }
    if (positions.size() == joint_names_.size()) {
        robot_state_->setVariablePositions(joint_names_, positions);
        has_received_joint_state_ = true;
    }
}

void HapticTeleopController::hapticPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_haptic_pose_msg_ = *msg;
    has_received_haptic_pose_ = true;
    if (is_active_) {
        calculateAndPublishTrajectory();
    }
}

void HapticTeleopController::buttonCallback(const geomagic_control::DeviceButtonEvent& msg)
{
    if (msg.grey_button == 1 && !is_active_) {
        if (recordInitialPoses()) {
            is_active_ = true;
            ROS_INFO("Teleoperation ACTIVATED (Incremental Mode).");
        } else {
            ROS_WARN("Failed to record initial poses. Teleoperation remains DEACTIVATED.");
        }
    } else if (msg.grey_button == 0 && is_active_) {
        is_active_ = false;
        ROS_INFO("Teleoperation DEACTIVATED.");
    }
}

bool HapticTeleopController::recordInitialPoses()
{
    if (!has_received_joint_state_) {
        ROS_ERROR("No joint state received yet. Cannot activate teleoperation.");
        return false;
    }
    last_robot_pose_ = robot_state_->getGlobalLinkTransform(robot_eef_frame_);
    if (!has_received_haptic_pose_) {
        ROS_WARN("Waiting for first haptic pose message...");
        ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/Geomagic/pose", ros::Duration(2.0));
        if (!has_received_haptic_pose_) {
            ROS_ERROR("Never received haptic pose message. Cannot activate.");
            return false;
        }
    }
    transformMsgToEigen(latest_haptic_pose_msg_.pose, last_haptic_pose_in_haptic_frame_);
    last_haptic_pose_in_haptic_frame_.translation() /= 1000.0;
    ROS_INFO("Initial and Last poses recorded successfully for teleoperation.");
    return true;
}

void HapticTeleopController::calculateAndPublishTrajectory()
{
    if (!has_received_haptic_pose_ || !has_received_joint_state_) return;
    Eigen::Isometry3d current_haptic_pose_in_haptic_frame;
    transformMsgToEigen(latest_haptic_pose_msg_.pose, current_haptic_pose_in_haptic_frame);
    current_haptic_pose_in_haptic_frame.translation() /= 1000.0;
    Eigen::Isometry3d haptic_delta = last_haptic_pose_in_haptic_frame_.inverse() * current_haptic_pose_in_haptic_frame;
    Eigen::Vector3d mapped;
    mapped.x() = -haptic_delta.translation().y();
    mapped.y() = haptic_delta.translation().x();
    mapped.z() = haptic_delta.translation().z();
    haptic_delta.translation() = mapped;
    haptic_delta.translation() *= linear_scale_;
    Eigen::AngleAxisd aa(haptic_delta.linear());
    aa.angle() *= angular_scale_;
    haptic_delta.linear() = aa.toRotationMatrix();
    // 修正了之前版本中的笔误 hdelta -> haptic_delta
    Eigen::Isometry3d target_robot_pose = last_robot_pose_ * haptic_delta;
    std::vector<double> joint_values;
    bool found_ik = robot_state_->setFromIK(joint_model_group_, target_robot_pose, 0.1);
    if (found_ik) {
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        trajectory_msgs::JointTrajectory traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.joint_names = joint_names_;
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = joint_values;
        point.time_from_start = ros::Duration(0.1);
        traj_msg.points.push_back(point);
        traj_pub_.publish(traj_msg);
        last_haptic_pose_in_haptic_frame_ = current_haptic_pose_in_haptic_frame;
        last_robot_pose_ = target_robot_pose;
    } else {
        ROS_WARN_THROTTLE(1.0, "IK solution not found for incremental step.");
    }
}

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "haptic_teleop_node");
    ros::NodeHandle nh; 

    while (!nh.hasParam("robot_description") && ros::ok())
    {
        ROS_INFO_THROTTLE(1.0, "Waiting for robot_description on parameter server...");
        ros::Duration(0.5).sleep();
    }

    HapticTeleopController controller(nh);
    
    controller.run();
    return 0;
}