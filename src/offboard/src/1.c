#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pd_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_position_cb);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 等待MAVROS连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 设定目标位置
    double target_x = 2.0, target_y = 2.0, target_z = 2.0;

    // PD控制器参数
    double kp = 1.3; // 比例系数
    double kd = 0.8; // 微分系数

    // PD控制器误差和上一次误差
    double error_x = 0.0, error_y = 0.0, error_z = 0.0;
    double error_last_x = 0.0, error_last_y = 0.0, error_last_z = 0.0;

    // 速率
    ros::Rate rate(20.0);

    while(ros::ok()){
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 计算误差
        error_x = target_x - current_pose.pose.position.x;
        error_y = target_y - current_pose.pose.position.y;
        error_z = target_z - current_pose.pose.position.z;

        // 计算速度
        double vel_x = kp * error_x + kd * (error_x - error_last_x);
        double vel_y = kp * error_y + kd * (error_y - error_last_y);
        double vel_z = kp * error_z + kd * (error_z - error_last_z);

        // 更新上一次误差
        error_last_x = error_x;
        error_last_y = error_y;
        error_last_z = error_z;

        // 发布速度
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.twist.linear.x = vel_x;
        vel_msg.twist.linear.y = vel_y;
        vel_msg.twist.linear.z = vel_z;
        local_vel_pub.publish(vel_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}