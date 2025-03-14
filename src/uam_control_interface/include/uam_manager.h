#ifndef UAM_MANAGER_H
#define UAM_MANAGER_H
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

namespace uam_control {
    class UamManager {
    public:

        using Vector5d = Eigen::Matrix<double, 5, 1>;

        UamManager();
        ~UamManager();

        /**
         * @brief takeoff the UAM and return while the UAM is in offboard mode and is armed.
         * It should followed by specific commands.
         * @param height the height of the UAM to takeoff (m)
         */
        void takeoff(double height);

        /**
         * @brief takeoff the UAM and return while the UAM is in offboard mode and is armed.
         * It should followed by specific commands.
         * use default takeoff height of 2.0 m.
         */
        void takeoff();

        /**
        * @brief set the target configuration of the UAM [q=(x,y,z,psi,theta)] in local NED frame
        * @param x target x-coordinate of the UAM (m)
        * @param y target y-coordinate of the UAM (m)
        * @param z target z-coordinate of the UAM (m)
        * @param psi target yaw of the UAM (rad)
        * @param theta target pitch of the UAM (rad)
        */
        void setTargetConfig(float x, float y, float z, float psi, float theta) const;

        /**
        * @brief set the target velocity of the UAM [dq=(dx,dy,dz,dpsi,dtheta)]
        * @param dx target x-velocity of the UAM (m/s)
        * @param dy target y-velocity of the UAM (m/s)
        * @param dz target z-velocity of the UAM (m/s)
        * @param dpsi target yaw rate of the UAM (rad/s)
        * @param theta target joint angle of the UAM (rad)
        *
        * @TODO: make the joint move in velocity control mode
        */
        void setTargetVel(double dx, double dy, double dz, double dpsi, double theta) const;

        /**
        * @brief get the current pose of the UAM [q=(x,y,z,psi,theta)] in local NED frame
        * @return a vector of 5 elements containing the pose of the UAM
        */
        Vector5d getConfig() const;

        /**
        * @brief get the current velocity of the UAM [dq=(dx,dy,dz,dpsi,dtheta)]
        * @return a vector of 5 elements containing the velocity of the UAM
        */
        Vector5d getVel() const;

        /**
        * @brief convert the quaternion to roll, pitch, yaw
        * @param p the pose of the UAM in the local NED frame
        * @return a vector of 3 elements containing the roll, pitch, and yaw of the UAM
        */
        static Eigen::Vector3d quadToRpy(const geometry_msgs::PoseStamped& p);

        // callback functions for subscribers

        /**
         * @brief callback function for state subscriber
        */
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);

        /**
         * @brief callback function for pose subscriber
        */
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * @brief callback function for twist subscriber
        */
        void velCallback(const geometry_msgs::Twist::ConstPtr& msg);

    private:
        // state related variables
        mavros_msgs::State current_state_;
        geometry_msgs::PoseStamped current_pose_;
        geometry_msgs::Twist current_vel_;
        double current_joint_pos_{0.0};

        // subscriber to get UAM information
        ros::NodeHandle nh_;
        ros::Subscriber state_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber vel_sub_;
        ros::Subscriber target_config_sub_; // get the target configuration of the UAM [q=(x,y,z,psi,theta)]

        // publisher to send UAM control commands
        ros::Publisher pose_pub_;
        ros::Publisher vel_pub_;
        ros::Publisher joint_pub_;
        ros::Publisher pose_test_pub_;

        // service clients to set UAM mode
        ros::ServiceClient set_mode_client_;
        ros::ServiceClient arm_client_;

        // factors for velocity control
        double Kp_pos_{0.05}, Kp_psi_{0.05};

        // parameters
        double joint_offset_ = -M_PI/6;
        double loop_rate_ = 20.0;
        // double takeoff_height_ = 2.0;
        double takeoff_height_ = 65.0;
        bool is_takeoff_{false};
    };
}// namespace uam_control

#endif //UAM_MANAGER_H
