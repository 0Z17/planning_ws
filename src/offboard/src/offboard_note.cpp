#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cmath>

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/PoseStamped.h>


#include <tf/tf.h>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Core>
#include </usr/include/eigen3/Eigen/StdVector>
#include </usr/include/eigen3/Eigen/Geometry>

// #include "offboard_pkg/info.h"
//#include <vtec_msgs/TrackingResult.h>
#include <ros/ros.h>

class GEOMETRY_CONTROL
{

public:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_, imu_sub_,velocity_sub_,position_sub_;
    bool IMUUpdateState,HomoRevState,VelocityUpdateState,ForceUpdateState,EzUpdateState,OffboardState,PositionUpdateState; 

    double mass_, gravity_, dt, ct;
    double  count_=0;
    Eigen::Matrix3d J_, I_3_3_;
    Eigen::Vector3d e3_;

    Eigen::Vector3d mavPos_, mavVel_, mavAcc_, mavAngularVel_;
    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d Pos_d_, Vel_d_, Acc_d_;
    Eigen::Vector4d Att_d_;
    Eigen::Matrix3d rotMat_;
    Eigen::Matrix3d rotMat_d_;
    Eigen::Matrix3d rotMat_d_last_;
    Eigen::Matrix3d rotMat_d_time_;
    Eigen::Vector3d angVel_d_, angVel_d_last_;
    double mavYaw_;
    double roll_, pitch_, yaw_;
    Eigen::Vector3d e_Pos_, e_Vel_, e_Acc_, e_rotMat_, e_angVel_, s_Pos_;
    Eigen::Vector3d b1d_m_, b1d_, b2d_, b3d_;
    Eigen::Vector3d b1d_m_time_, b1d_time_, b2d_time_, b3d_time_;

    Eigen::Vector3d thrEstimate_, zetaPos_, zetaPos_time_;
    double gamma_p_;
    Eigen::Vector3d torqueEstimate_, zetaAtt_, zetaAtt_time_;
    double gamma_a_, h2_;

    double thrust_;
    Eigen::Vector3d thrustVir_, torque_;
    Eigen::Vector3d thrustVir_last_, thrustVir_time_;
    Eigen::Vector3d rate_;
    Eigen::Matrix3d Kpos_, Kvel_, KR_, Kw_, Lambda_;
    
    ros::Time t_now, t_last;

public:
    GEOMETRY_CONTROL()
    {
        imu_sub_ = nh_.subscribe("/mavros/imu/data", 10, &GEOMETRY_CONTROL::imuCallback, this);
        position_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &GEOMETRY_CONTROL::mavposeCallback, this);
        velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &GEOMETRY_CONTROL::mavtwistCallback, this);
        

        mass_ = 2.1077;
        gravity_ = 9.81;
        dt = 0.005;
        ct = 0.0;
        J_ << 0.029125,0,0, 0,0.029125,0, 0,0,0.055225;
        e3_ << 0,0,1;
        
        mavPos_ = Eigen::Vector3d::Zero();
        mavVel_ = Eigen::Vector3d::Zero();
        mavAcc_ = Eigen::Vector3d::Zero();
        mavAngularVel_ = Eigen::Vector3d::Zero();
        mavAtt_ = Eigen::Vector4d::Zero();
        Pos_d_ = Eigen::Vector3d::Zero();
        Vel_d_ = Eigen::Vector3d::Zero();
        Acc_d_ = Eigen::Vector3d::Zero();
        Att_d_= Eigen::Vector4d::Zero();
        rotMat_ = Eigen::Matrix3d::Identity();
        rotMat_d_ = Eigen::Matrix3d::Identity();
        rotMat_d_last_ = Eigen::Matrix3d::Identity();
        rotMat_d_time_ = Eigen::Matrix3d::Zero();
        angVel_d_ = Eigen::Vector3d::Zero();
        angVel_d_last_ = Eigen::Vector3d::Zero();
        mavYaw_ = 0;
        roll_ = 0;   pitch_ = 0;  yaw_ = 0;
        e_Pos_ = Eigen::Vector3d::Zero();
        e_Vel_ = Eigen::Vector3d::Zero();
        e_Acc_ = Eigen::Vector3d::Zero();
        e_rotMat_ = Eigen::Vector3d::Zero();
        e_angVel_ = Eigen::Vector3d::Zero();
        s_Pos_= Eigen::Vector3d::Zero();
        b1d_m_ = Eigen::Vector3d::Zero();
        b1d_ = Eigen::Vector3d::Zero();
        b2d_ = Eigen::Vector3d::Zero();
        b3d_ = Eigen::Vector3d::Zero();
        b1d_m_time_ = Eigen::Vector3d::Zero();
        b1d_time_ = Eigen::Vector3d::Zero();
        b2d_time_ = Eigen::Vector3d::Zero();
        b3d_time_ = Eigen::Vector3d::Zero();

        thrEstimate_ = Eigen::Vector3d::Zero();
        zetaPos_ = Eigen::Vector3d::Zero();
        zetaPos_time_ = Eigen::Vector3d::Zero();
        gamma_p_ = 0.1;

        torqueEstimate_ = Eigen::Vector3d::Zero();
        zetaAtt_ = Eigen::Vector3d::Zero();
        zetaAtt_time_ = Eigen::Vector3d::Zero();
        gamma_a_ = 0.05;
        h2_ = 0.01;
        

        thrust_ = mass_ * gravity_;
        thrustVir_ = Eigen::Vector3d::Zero();
        torque_ = Eigen::Vector3d::Zero();
        thrustVir_last_ = Eigen::Vector3d::Zero();
        thrustVir_time_ = Eigen::Vector3d::Zero();
        rate_ = Eigen::Vector3d::Zero();
        // Kpos_ << 2,0,0, 0,2,0, 0,0,4;
        // Kvel_ << 0.1,0,0, 0,0.1,0, 0,0,5;
        Lambda_ << 5,0,0, 0,5,0, 0,0,5;
        //Kpos_ << 1.47,0,0, 0,1.47,0, 0,0,5.47;
        //Kvel_ << 1.47,0,0, 0,1.47,0, 0,0,4.47;
        Kpos_ << 2.315,0,0, 0,2.117,0, 0,0,4.217;
        Kvel_ << 2.114,0,0, 0,1.851,0, 0,0,3.612 ;
        KR_ << 1.0,0,0, 0,1.1,0, 0,0,1.0;
        Kw_ << 0.09,0,0, 0,0.1,0, 0,0,0.09;
        // Lambda_ << 5,0,0, 0,5,0, 0,0,5;
        IMUUpdateState = false;
        HomoRevState = false;
        VelocityUpdateState = false;
        ForceUpdateState = false;
        EzUpdateState = false;
        OffboardState = true;
        PositionUpdateState = false;
        /* ****************************** */
        
    }
    ~GEOMETRY_CONTROL(){

    }

    // update the imu data to calculate the rotation matrix
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        sensor_msgs::Imu imu_data_ = *msg;

        mavAtt_(0) = imu_data_.orientation.w;
        mavAtt_(1) = imu_data_.orientation.x;
        mavAtt_(2) = imu_data_.orientation.y;
        mavAtt_(3) = imu_data_.orientation.z;

        rotMat_ = quat2RotMatrix(mavAtt_);
        std::cout << "rotmat: " << rotMat_<< std::endl;
        mavAngularVel_(0) = imu_data_.angular_velocity.x;
        mavAngularVel_(1) = imu_data_.angular_velocity.y;
        mavAngularVel_(2) = imu_data_.angular_velocity.z;

    }

    // get local position.
    void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        
        geometry_msgs::PoseStamped pos_data = *msg;
        mavPos_ << pos_data.pose.position.x, pos_data.pose.position.y, pos_data.pose.position.z;
        // mavAtt_(0) = pos_data.pose.orientation.w;
        // mavAtt_(1) = pos_data.pose.orientation.x;
        // mavAtt_(2) = pos_data.pose.orientation.y;
        // mavAtt_(3) = pos_data.pose.orientation.z;
;
        // rotMat_ = quat2RotMatrix(mavAtt_);

        PositionUpdateState = true;
       

    }

  
   // get Velocity in the base_link frame.
    void mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        geometry_msgs::TwistStamped odom = *msg; // base_link
            
        mavVel_(0) = odom.twist.linear.x;
        mavVel_(1) = odom.twist.linear.y;
        mavVel_(2) = odom.twist.linear.z;
        // mavAngularVel_(0) = odom.twist.angular.x;
        // mavAngularVel_(1) = odom.twist.angular.y;
        // mavAngularVel_(2) = odom.twist.angular.z;

        // std::cout << "w: " << mavAngularVel_ << std::endl;

        VelocityUpdateState = true;  

    }


    //处理进程
    void Process()
    {
        targetTrajectory();
        posController();
        targetAtt();
        attController();
    }
// Target
    void targetTrajectory(){
        Pos_d_ << 0.0,0.0,5.0;
        // Pos_d_ << 0.43,1.78,0.5;
        Vel_d_ << 0,0,0;
        Acc_d_ << 0,0,0;
    }
    // position controller
    void posController()
    {
        e_Pos_ = mavPos_ -Pos_d_;
        e_Vel_ = mavVel_ - Vel_d_;
        s_Pos_ = e_Vel_ + Lambda_ * e_Pos_;

        // Pos observer
        thrEstimate_ = zetaPos_ ;
        //thrEstimate_ = zetaPos_ + gamma_p_ * s_Pos_;
        Eigen::Vector3d a_fb = -Kpos_ * e_Pos_ -Kvel_ * e_Vel_ ;
        //Eigen::Vector3d a_fb = - Kpos_ * s_Pos_ - mass_* Lambda_ * e_Vel_;
        // Eigen::Vector3d a_fb = - Kpos_ * s_Pos_ - mass_* Lambda_ * e_Vel_ - thrEstimate_;
        if (a_fb.norm() > 9){
            a_fb = (9 / a_fb.norm()) * a_fb;
        }
        std::cout << "a_fb: " << a_fb << std::endl;
        thrustVir_ = mass_*a_fb + mass_ * Acc_d_ + mass_ * gravity_ * e3_ - thrEstimate_;
        //std::cout << "thrEstimate_: " << thrEstimate_ << std::endl;
        //thrustVir_ = a_fb + mass_ * Acc_d_ + mass_ * gravity_ * e3_;
        //thrust_ = (thrustVir_).transpose() * rotMat_ * e3_;
        //std::cout << "thrust: " << thrustVir_ << std::endl;
        // observer update
        zetaPos_time_ = -gamma_p_ * ( thrustVir_ - mass_ * gravity_ * e3_ - mass_ * Acc_d_ + Kpos_ * e_Pos_ + Kvel_ * e_Vel_  + thrEstimate_);
        //zetaPos_time_ = -gamma_p_ * ( thrustVir_ - mass_ * gravity_ * e3_ - mass_ * Acc_d_ + mass_ * Lambda_ * e_Vel_ + thrEstimate)+ s_Pos_;
        zetaPos_ = zetaPos_ + zetaPos_time_ * dt;
        //std::cout << "e_pos_x: " << e_Pos_<< std::endl;
        //std::cout << "thrustVir_" << thrustVir_ << std::endl;
        //std::cout << "pos_x: " << mavPos_[0]<< std::endl;
        //std::cout << "pos_y: " << mavPos_[1]<< std::endl;
        //std::cout << " e_Pos_: " <<  e_Pos_ << std::endl;
    }

    // calculate desired attitude
    void targetAtt(){
       
        // Desired R_d
        mavYaw_ = 0;
        // b1d_m_ << cos(mavYaw_), sin(mavYaw_), 0;
        // b1d_m_ << 1,0,0;
        // b3d_ = thrustVir_ / (rostopic echo /mavros/local_position/posethrustVir_.norm());
        // b3d_ = b3d_ / b3d_.norm();
        // b2d_ = b3d_.cross(b1d_m_);
        // b2d_ = b2d_ / b2d_.norm();
        // b1d_ = b2d_.cross(b3d_);
        // b1d_ = b1d_ / b1d_.norm();

        //b1d_ << 1, 0, 0;
        //b2d_ << 0, 1, 0;
        //b3d_ << 0, 0, 1;
        //xuanzhan10du 
        
        t_now = ros::Time::now();
        dt = (t_now - t_last).toSec() ;
        //std::cout << "dt: " << dt << std::endl;
        count_=count_+1;
        std::cout<<"count_=" << count_ << std::endl;
        if (count_ > 3500)
        {
        b1d_ << 1, 0, 0;
        b2d_ << 0, 1, 0;
        b3d_ << 0, 0, 1;

        //b1d_ << 1, 0, 0;
        //b2d_ << 0,0.9848,-0.1736;
        //b3d_ << 0, 0.1736, 0.9848;
        }
        else
        {
            b1d_ << 1, 0, 0;
            b2d_ << 0, 1, 0;
            b3d_ << 0, 0, 1;
        }

        // Desired Angular Velocity
        // Eigen::Matrix3d trans_vector0;
        // trans_vector0 = rotMat_d_ * (rotMat_d_last_.transpose()) - I_3_3_;
        rotMat_d_ << b1d_(0),b2d_(0),b3d_(0), b1d_(1),b2d_(1),b3d_(1), b1d_(2),b2d_(2),b3d_(2);
         Att_d_ = rot2Quaternion(rotMat_d_);

        angVel_d_ = vex_matrix((rotMat_d_ * (rotMat_d_last_.transpose()) - I_3_3_) / dt);
        
        rotMat_d_last_ = rotMat_d_;
        t_last = t_now;
        // std::cout << "A: " << trans_vector0 << std::endl;
       // std::cout << "wd: " << (rotMat_d_ * (rotMat_d_last_.transpose()) - I_3_3_) / dt << std::endl;   
        
    }

    // attitude controlller
    void attController(){
        Eigen::Vector3d p1,p2,p3,p4;
        e_rotMat_ = 0.5 * vex_matrix(rotMat_d_.transpose() * rotMat_ - rotMat_.transpose() * rotMat_d_);
        e_angVel_ = mavAngularVel_ - rotMat_.transpose() * rotMat_d_ * angVel_d_;

        torqueEstimate_ = zetaAtt_ + gamma_a_ * J_ * e_angVel_;

        p1 = - KR_ * e_rotMat_;
        p2 = - Kw_ * e_angVel_;
        p3 = matrix_vex(mavAngularVel_) * J_ * mavAngularVel_;
        p4 = - J_ * (matrix_vex(mavAngularVel_) * rotMat_.transpose() * rotMat_d_ * angVel_d_ - rotMat_.transpose() * rotMat_d_ * (angVel_d_ - angVel_d_last_) / dt);
        
        torque_ = p1 + p2;
        // torque_ = p1 + p2 - torqueEstimate_;

        // std::cout << "p1" << p1 << std::endl;
        // std::cout << "p2" << p2 << std::endl;
        // std::cout << "p3" << p3 << std::endl;
        angVel_d_last_ = angVel_d_;
        // observer
        zetaAtt_time_ = -gamma_a_ * (torque_ + torqueEstimate_ - p3 + p4) + (h2_ * J_.inverse() * e_rotMat_ + e_angVel_);
        zetaAtt_ = zetaAtt_ + zetaAtt_time_ * dt;
    }
    

    


    Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &Rot) {
        Eigen::Vector4d uav_quat;
        double tr = Rot.trace();
        if (tr > 0.0) {
            double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
            uav_quat(0) = 0.25 * S;
            uav_quat(1) = (Rot(2, 1) - Rot(1, 2)) / S;
            uav_quat(2) = (Rot(0, 2) - Rot(2, 0)) / S;
            uav_quat(3) = (Rot(1, 0) - Rot(0, 1)) / S;
        } else if ((Rot(0, 0) > Rot(1, 1)) & (Rot(0, 0) > Rot(2, 2))) {
            double S = sqrt(1.0 + Rot(0, 0) - Rot(1, 1) - Rot(2, 2)) * 2.0;  // S=4*qx
            uav_quat(0) = (Rot(2, 1) - Rot(1, 2)) / S;
            uav_quat(1) = 0.25 * S;
            uav_quat(2) = (Rot(0, 1) + Rot(1, 0)) / S;
            uav_quat(3) = (Rot(0, 2) + Rot(2, 0)) / S;
        } else if (Rot(1, 1) > Rot(2, 2)) {
            double S = sqrt(1.0 + Rot(1, 1) - Rot(0, 0) - Rot(2, 2)) * 2.0;  // S=4*qy
            uav_quat(0) = (Rot(0, 2) - Rot(2, 0)) / S;
            uav_quat(1) = (Rot(0, 1) + Rot(1, 0)) / S;
            uav_quat(2) = 0.25 * S;
            uav_quat(3) = (Rot(1, 2) + Rot(2, 1)) / S;
        } else {
            double S = sqrt(1.0 + Rot(2, 2) - Rot(0, 0) - Rot(1, 1)) * 2.0;  // S=4*qz
            uav_quat(0) = (Rot(1, 0) - Rot(0, 1)) / S;
            uav_quat(1) = (Rot(0, 2) + Rot(2, 0)) / S;
            uav_quat(2) = (Rot(1, 2) + Rot(2, 1)) / S;
            uav_quat(3) = 0.25 * S;
        }
    return uav_quat;
    }

    Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
        Eigen::Matrix3d rotmat;
        rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
        
        return rotmat;
    }

    Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
        Eigen::Vector4d quat;
        quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  
        return quat;
    }


    Eigen::Vector3d vex_matrix(Eigen::Matrix3d S)
    {
        Eigen::Vector3d v;

        v << 0.5 * (S(2,1) - S(1,2)), 0.5 * (S(0,2) - S(2,0)), 0.5 * (S(1,0) - S(0,1));

        return v;
        
    }

    Eigen::Matrix3d matrix_vex(Eigen::Vector3d v)
    {
        Eigen::Matrix3d mat;
        mat << 0, -v(2), v(1), 
               v(2), 0, -v(0), 
               -v(1), v(0), 0;
        // std::cout<<"mat: "<<std::endl; std::cout<<mat<<std::endl;
        return mat;
    }


};

mavros_msgs::State current_state;
void mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    GEOMETRY_CONTROL geometry_control;// class

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, mavstateCallback);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_poistion/local",10);
    ros::Publisher rate_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude",10);
    // ros::Publisher uav_info = nh.advertise<offboard_pkg::info>("/uav_info",10);
    ros::Publisher actuatorControl_pub_ = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control",10);
    

    geometry_msgs::PoseStamped cmdPose_;
    mavros_msgs::AttitudeTarget cmdRate_;

    mavros_msgs::ActuatorControl cmd;


  //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(200.0);
    double f_x_,f_y_,f_z,f_xy,max_f_xy,max_f_x,max_f_y,f_z_,f_;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    bool take_off = false;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    while(ros::ok()){
        
            geometry_control.Process();

            cmdPose_.header.stamp = ros::Time::now();
            cmdPose_.header.frame_id = "map";
            cmdPose_.pose.position.x = geometry_control.Pos_d_(0);
            cmdPose_.pose.position.y = geometry_control.Pos_d_(1);
            cmdPose_.pose.position.z = geometry_control.Pos_d_(2);
            cmdPose_.pose.orientation.w = geometry_control.Att_d_(0);
            cmdPose_.pose.orientation.x = geometry_control.Att_d_(1);
            cmdPose_.pose.orientation.y = geometry_control.Att_d_(2);
            cmdPose_.pose.orientation.z = geometry_control.Att_d_(3);


            // cmd.header.stamp = ros::Time::now();
            // cmd.header.frame_id = "map";
            f_x_ = geometry_control.thrustVir_(0);
            f_y_= geometry_control.thrustVir_(1);
            f_z_= geometry_control.thrustVir_(2);
            f_ = sqrt(f_x_*f_x_+f_y_*f_y_+f_z_*f_z_);
            cmd.group_mix = 0;
            cmd.controls[0] = std::max(-0.3, std::min(0.3, geometry_control.torque_(0)));
            cmd.controls[1] = std::max(-0.3, std::min(0.3, -geometry_control.torque_(1)));
            cmd.controls[2] = std::max(-0.3, std::min(0.3, -geometry_control.torque_(2)));
            //cmd.controls[3] = std::max(0.0, std::min(0.95, 0.05*(geometry_control.thrustVir_(2)-geometry_control.mass_*geometry_control.gravity_)+0.83));
            cmd.controls[3] = std::max(0.0, std::min(0.9, 0.65 *(f_/20.6))); 
            cmd.controls[4] = 0.0;
            cmd.controls[5] = 0.0;
            cmd.controls[6] = 0.0;
            cmd.controls[7] = 0.0;

            f_xy=sqrt(f_x_*f_x_+f_y_*f_y_);
           
            f_z=-std::max(0.06, std::min(0.95, 0.62*(geometry_control.thrustVir_(2)/20.6)));
            max_f_xy=sqrt(0.95*0.95-f_z*f_z);
            //max_f_x=max_f_xy*(f_x_/ f_xy);
            //max_f_y=max_f_xy*(f_y_/ f_xy);

            max_f_x = max_f_xy * (std::abs(f_x_) / std::abs(f_xy));
            max_f_y = max_f_xy * (std::abs(f_y_) / std::abs(f_xy));
            std::cout << "max_f_x: " << max_f_x << std::endl;
            cmd.controls[8] = std::max(-max_f_x, std::min(max_f_x, (geometry_control.thrustVir_(0)/20.6*0.65)+0.135));
            //cmd.controls[8] = -std::max(-max_f_x, std::min(max_f_x, (geometry_control.thrustVir_(0)/20.6*0.65)));
            cmd.controls[9] = -std::max(-max_f_xy, std::min(max_f_y, (geometry_control.thrustVir_(1)/20.6*0.65)+0.025));
            //cmd.controls[10] = -std::max(0.0, std::min(0.95, 0.01*(geometry_control.thrustVir_(2)-geometry_control.mass_*geometry_control.gravity_)+0.83));
            //cmd.controls[10] = -std::max(0.0, std::min(0.95, 0.01*(geometry_control.thrustVir_(2)-geometry_control.mass_*geometry_control.gravity_)+0.83));
            cmd.controls[10] = -std::max(0.0, std::min(0.95, 0.639*(geometry_control.thrustVir_(2)/20.6)));
            
            //cmd.controls[8] = std::max(0.0, std::min(max_f_xy, max_f_xy*(f_x / f_xy)));
            //cmd.controls[9] = std::max(0.0, std::min(max_f_xy, max_f_xy*(f_y / f_xy)));
            //cmd.controls[8] = 0;
            //cmd.controls[9] = 0;
            // uav_msg.wd_x = geometry_control.angVel_d_(0);
            // uav_msg.wd_y = geometry_control.angVel_d_(1);
            // uav_msg.wd_z = geometry_control.angVel_d_(2);
            // uav_msg.Torque_x =  geometry_control.torque_(0);
            // uav_msg.Torque_y = -geometry_control.torque_(1);
            // uav_msg.Torque_z = -geometry_control.torque_(2);
            // uav_msg.thrust = geometry_control.thrust_;
            
            // uav_msg.Thrust_vir_x = geometry_control.thrustVir_(0);
            // uav_msg.Thrust_vir_y = geometry_control.thrustVir_(1);
            // uav_msg.Thrust_vir_z = geometry_control.thrustVir_(2);

            // uav_msg.e_pos_x = geometry_control.e_Pos_(0);
            // uav_msg.e_pos_y = geometry_control.e_Pos_(1);
            // uav_msg.e_pos_z = geometry_control.e_Pos_(2);
            // uav_msg.e_vel_x = geometry_control.e_Vel_(0);
            // uav_msg.e_vel_y = geometry_control.e_Vel_(1);
            // uav_msg.e_vel_z = geometry_control.e_Vel_(2);



            // std::cout<<"mix_u.controls[0]=" << cmd.controls[0] << std::endl;
            // std::cout<<"mix_u.controls[1]=" << cmd.controls[1] << std::endl;
            // std::cout<<"mix_u.controls[2]=" << cmd.controls[2] << std::endl;
            // std::cout<<"mix_u.controls[3]=" << cmd.controls[3] << std::endl;
            // uav_info.publish(uav_msg);
            // pose_pub_.publish(cmdPose_);

             if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                    {
                    ROS_INFO("Offboard enabled");
                    take_off = true;
                    }
                    last_request = ros::Time::now();
                    }
                else
                    {
                     if (!current_state.armed &&
                        (ros::Time::now() - last_request > ros::Duration(2.0)))
                        {
                        if (arming_client.call(arm_cmd) &&
                                arm_cmd.response.success)
                            {
                             ROS_INFO("Vehicle armed");
 
                            last_request = ros::Time::now();
                            }
                        }
                    }
        if (take_off )
      {
        
        actuatorControl_pub_.publish(cmd);
        //std::cout << "cmd_wx: " << cmd.controls[0] << std::endl;
        std::cout << "cmd_x: " << cmd.controls[8] << std::endl;
        std::cout << "cmd_y: " << cmd.controls[9] << std::endl;
        std::cout << "cmd_z: " << cmd.controls[10] << std::endl;
    }   
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
