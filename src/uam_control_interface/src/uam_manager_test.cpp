#include "uam_manager.h"
#include "std_msgs/Float32MultiArray.h"

namespace uc = uam_control;

double target_x = 0.0;
double target_y = 0.0;
double target_z = 1.2;
double target_psi = 0.0;
double target_theta = 0.0;

double Kp_pos = 0.5;
double Kp_ang = 0.5;

void contact_point_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    target_x = msg->data[0];
    target_y = msg->data[1];
    target_z = msg->data[2];
    target_psi = msg->data[3];
    target_theta = msg->data[4];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "uam_manager_test");
    ros::NodeHandle nh;
    uc::UamManager uam_manager;
    uam_manager.takeoff();

    ros::Rate rate(100);

    ros::Subscriber contact_point_sub = nh.subscribe("target_point", 1, &contact_point_callback);

    ros::Time last_time = ros::Time::now();

    // get the waypoint from thr ros parameter server
    double waypoint_1_x, waypoint_1_y, waypoint_1_z;
    ros::param::get("/control_node/ref_p_x", waypoint_1_x);
    ros::param::get("/control_node/ref_p_y", waypoint_1_y);
    ros::param::get("/control_node/ref_p_z", waypoint_1_z);
    waypoint_1_x = waypoint_1_x - 0.1;
    double waypoint_1_psi = 0.0;
    double waypoint_1_theta = 0.0;

    // double waypoint_1_x = 0.746377 - 0.1;
    // double waypoint_1_y = 1.84411;
    // double waypoint_1_z = 1.19919;
    // double waypoint_1_psi = 0.0925802;
    // double waypoint_1_theta = -0.43517587559829884;

    double dist_1 = sqrt(pow(waypoint_1_x - target_x, 2) + pow(waypoint_1_y - target_y, 2) + pow(waypoint_1_z - target_z, 2));

    while(ros::ok() /** && abs(dist_1) > 0.1 */) {
        ROS_INFO("Distance to waypoint 1: %f", dist_1);
        uam_manager.setTargetConfig(waypoint_1_x, waypoint_1_y, waypoint_1_z, waypoint_1_psi, waypoint_1_theta);
        Eigen::Vector3d pos_current = uam_manager.getConfig().head(3);
        dist_1 = sqrt(pow(waypoint_1_x - pos_current[0], 2) + pow(waypoint_1_y - pos_current[1], 2) + pow(waypoint_1_z - pos_current[2], 2));
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("///////////////////////////////////");
    ROS_INFO("Reached waypoint 1");
    ROS_INFO("///////////////////////////////////");

    double waypoint_2_x = 1.2;
    double waypoint_2_y = 0.0;
    double waypoint_2_z = 1.5;

    double dist_2 = sqrt(pow(waypoint_1_x - target_x, 2) + pow(waypoint_1_y - target_y, 2) + pow(waypoint_1_z - target_z, 2));

    while(ros::ok() &&  abs(dist_2) > 0.02) {
        ROS_INFO("Distance to waypoint 2: %f", dist_2);
        uam_manager.setTargetConfig(waypoint_2_x, waypoint_2_y, waypoint_2_z, 0.0, 0.0);
        Eigen::Vector3d pos_current = uam_manager.getConfig().head(3);
        dist_2 = sqrt(pow(waypoint_2_x - pos_current[0], 2) + pow(waypoint_2_y - pos_current[1], 2) + pow(waypoint_2_z - pos_current[2], 2));
        ros::spinOnce();
        rate.sleep();
    }


    // Wait for contact point message
    while(ros::ok()){
        uc::UamManager::Vector5d pos_current = uam_manager.getConfig();
        double x_error = target_x - pos_current[0];
        double y_error = target_y - pos_current[1];
        double z_error = target_z - pos_current[2];
        double psi_error = target_psi - pos_current[3];
 
        double x_vel = Kp_pos * x_error;
        double y_vel = Kp_pos * y_error;
        double z_vel = Kp_pos * z_error;
        double psi_vel = Kp_ang * psi_error;

        uam_manager.setTargetVel(x_vel, y_vel, z_vel, psi_vel, target_theta);

        if (ros::Time::now() - last_time > ros::Duration(1.0)) {
            ROS_INFO("Config Error: (%f, %f, %f, %f, %f)", x_error, y_error, z_error, psi_error, target_theta);
            last_time = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}