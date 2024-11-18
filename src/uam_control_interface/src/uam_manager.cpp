#include "uam_manager.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>

namespace uc = uam_control;
using namespace uc;

UamManager::UamManager()
{
    // initialize subscribers
    state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 1, &UamManager::stateCallback, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &UamManager::poseCallback, this);
    vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("mavros/local_position/velocity", 1, &UamManager::velCallback, this);

    // initialize publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    joint_pub_ = nh_.advertise<std_msgs::Float32>("skyvortex/operator_1_joint/pos_cmd", 10);

    pose_test_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // initialize client
    arm_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

UamManager::~UamManager() = default;

void UamManager::takeoff(double height) {
    ros::Rate loop_rate(loop_rate_);

    // wait for FCU connection
    while(ros::ok() && current_state_.connected){
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        setTargetConfig(0.0, 0.0, height, 0.0, 0.0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // set mode to offboard and arm the UAM
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state_.armed){
        if( current_state_.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO(current_state_.mode.c_str());
            if(set_mode_client_.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
                if((ros::Time::now() - last_request > ros::Duration(5.0))){
                    ROS_INFO("arming");
                    if( arm_client_.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
        }
        setTargetConfig(0.0, 0.0, height, 0.0, 0.0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // send command until reaching the desired height
    while(ros::ok() && abs(current_pose_.pose.position.z - height) > 0.1) {
        setTargetConfig(0.0, 0.0, height, 0.0, 0.0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("UAM takeoff complete");

    is_takeoff_ = true;
}

void UamManager::takeoff() {
    takeoff(takeoff_height_);
}

void UamManager::setTargetConfig(float x, float y, float z, float psi, float theta)const {
    geometry_msgs::PoseStamped local_pose_msg;
    local_pose_msg.header.stamp = ros::Time::now();
    local_pose_msg.header.frame_id = "map";
    local_pose_msg.pose.position.x = x;
    local_pose_msg.pose.position.y = y;
    local_pose_msg.pose.position.z = z;

    local_pose_msg.pose.orientation.x = 0;
    local_pose_msg.pose.orientation.y = 0;
    local_pose_msg.pose.orientation.z = sin(psi/2);
    local_pose_msg.pose.orientation.w = cos(psi/2);

    pose_test_pub_.publish(local_pose_msg);

    std_msgs::Float32 joint_msg;
    joint_msg.data = theta + joint_offset_;
    joint_pub_.publish(joint_msg);
}

void UamManager::setTargetVel(const double dx, const double dy, const double dz,
                              const double dpsi, const double dtheta) const {
    geometry_msgs::TwistStamped local_vel_msg;
    local_vel_msg.header.stamp = ros::Time::now();
    local_vel_msg.header.frame_id = "map";
    local_vel_msg.twist.linear.x = dx;
    local_vel_msg.twist.linear.y = dy;
    local_vel_msg.twist.linear.z = dz;
    local_vel_msg.twist.angular.x = 0.0;
    local_vel_msg.twist.angular.y = 0.0;
    local_vel_msg.twist.angular.z = dpsi;

    vel_pub_.publish(local_vel_msg);

    std_msgs::Float32 joint_vel;
    joint_vel.data = dtheta + joint_offset_;
    joint_pub_.publish(joint_vel);
}

UamManager::Vector5d UamManager::getConfig() const {
    Eigen::Vector3d rpy = quadToRpy(current_pose_);
    Vector5d vec;
    vec << current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z,
          rpy[2], current_joint_pos_;

    return vec;
}

UamManager::Vector5d UamManager::getVel() const {
    Vector5d vec;
    vec << current_vel_.linear.x, current_vel_.linear.y, current_vel_.linear.z,
          current_vel_.angular.z, 0.0;

    return vec;
}


Eigen::Vector3d UamManager::quadToRpy(const geometry_msgs::PoseStamped& p) {
    const geometry_msgs::Quaternion quad = p.pose.orientation;
    double roll, pitch, yaw;
    const tf2::Quaternion q(quad.x, quad.y, quad.z, quad.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}


void UamManager::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void UamManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

void UamManager::velCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    current_vel_ = *msg;
}



