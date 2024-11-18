/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>

#define FLIGHT_ALTITUDE 1.5f

double target_x{0.0}, target_y{0.0}, target_z{2.0}, target_psi{0.0}, target_theta{-M_PI/6};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void set_target_pose(const ros::Publisher& local_pos_pub, const ros::Publisher& joint_pub,
                     const float x, const float y, const float z, const float psi, const float theta) {
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

    local_pos_pub.publish(local_pose_msg);

    std_msgs::Float32 joint_msg;
    joint_msg.data = theta;
    joint_pub.publish(joint_msg);
}

void target_pose_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    target_x = msg->data[0];
    target_y = msg->data[1];
    target_z = msg->data[2];
    target_psi = msg->data[3];
    target_theta = msg->data[4];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float32>
            ("skyvortex/operator_1_joint/pos_cmd", 10);
    ros::Subscriber target_pose_sub = nh.subscribe<std_msgs::Float32MultiArray>
            ("target_point", 10, target_pose_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        set_target_pose(local_pos_pub, joint_pub, 0, 0, FLIGHT_ALTITUDE, 0, 0);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        set_target_pose(local_pos_pub, joint_pub, 0, 0, 10.0, 0, 0);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}