#include <ros/ros.h>
#include "invkin.h"
#include <sensor_msgs/PointCloud2.h>
#include "VizTool.h"
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>

namespace sr = surface_reconstructor;
namespace dp = dynamic_planning;

class ContactManager
{
public:
    explicit ContactManager(const ros::NodeHandle& nh) {
        surface_ = nullptr;
        invkin_ = nullptr;
        nh_ = nh;
        ROS_INFO("ContactManager initialized");
        sub_ = nh_.subscribe("/filtered_point_cloud", 1, &ContactManager::contactCallback, this);
        sub_point_ = nh_.subscribe("/viz_point", 1, &ContactManager::getPointCallback, this);
        ROS_INFO("Creating contacts subscriber");
    }

    void contactCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        ROS_INFO("Contact callback");
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Initialize/update and fit the surface
        if (surface_ == nullptr) {
            ROS_INFO("Initializing surface");
            surface_ = new sr::Nurbs(cloud);
        }
        else {
            surface_->updatePointCloud(cloud);
        }
        surface_->fitSurface();

        // Initialize/update the inverse kinematics solver
        if (invkin_ == nullptr) {
            ROS_INFO("Initializing invkin");
            invkin_ = new dp::InvKin(surface_);
        }
        else {
            invkin_->updateSurface(surface_);
        }
    }

    void getPointCallback(const geometry_msgs::Point::ConstPtr& msg) {
        u_ = msg->x;
        v_ = msg->y;
    }

    static void frameTransform(Eigen::Vector3d& point_in, Eigen::Vector3d& point_out,
                               const geometry_msgs::TransformStamped& transform) {
        geometry_msgs::Point point_in_m, point_out_m;
        point_in_m.x = point_in[0];
        point_in_m.y = point_in[1];
        point_in_m.z = point_in[2];

        tf2::doTransform(point_in_m, point_out_m, transform);

        point_out[0] = point_out_m.x;
        point_out[1] = point_out_m.y;
        point_out[2] = point_out_m.z;
    }

    sr::Nurbs* getSurface() const {
        return surface_;
    }

    Eigen::Vector2d getPoint() const {
        return {u_, v_};
    }

    dp::InvKin* getInvKin() const {
        return invkin_;
    }

private:
    sr::Nurbs* surface_;
    dp::InvKin* invkin_;
    ros::NodeHandle nh_;
    double u_{0}, v_{0};
    ros::Subscriber sub_, sub_point_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "contact_test");
    ros::NodeHandle nh;

    // double rate;
    const ContactManager cm(nh);
    ROS_INFO("ContactManager initialized");

    // wait for the first contact callback
    while (cm.getSurface() == nullptr) {
        // ROS_INFO("Waiting for the surface to be initialized");
        ros::spinOnce();
    }


    cm.getSurface()->fitSurface();
    sr::VizTool viz(cm.getSurface());


    // nh.getParam("rate", rate);
    ros::Rate loop_rate(0.1);

    const ros::Publisher pub_surface = nh.advertise<visualization_msgs::Marker>("surface_marker", 1);
    const ros::Publisher pub_point = nh.advertise<visualization_msgs::Marker>("point_marker", 1);
    const ros::Publisher pub_normal = nh.advertise<visualization_msgs::Marker>("normal_marker", 1);
    const ros::Publisher pub_base_point = nh.advertise<visualization_msgs::Marker>("base_point", 1);
    const ros::Publisher target_pub = nh.advertise<std_msgs::Float32MultiArray>("target_point", 1);

    // initialize tf listener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::TransformStamped end_to_optical, optical_to_map;

    ros::Duration(3.0).sleep();

    while (ros::ok()) {
        ros::spinOnce();

        end_to_optical = tf_buffer.lookupTransform( "depth_optical_frame", "end_Link", ros::Time(0));
        optical_to_map = tf_buffer.lookupTransform("map", "depth_optical_frame", ros::Time(0));

        // get the end point in the optical frame
        Eigen::Vector3d n_ori_in_optical, n_end_in_optical;
        Eigen::Vector3d n_ori_in_map, n_end_in_map;
        Eigen::Vector3d end_in_optical;
        Eigen::Vector3d end_in_end_frame(0, 0, 0);

        ContactManager::frameTransform(end_in_end_frame, end_in_optical, end_to_optical);
        double u_closest, v_closest;
        cm.getSurface()->getClosestPoint(end_in_optical, u_closest, v_closest);
        dp::Vector6d qs = cm.getInvKin()->xToQs(u_closest, v_closest);
        n_ori_in_optical(0) = qs(0);
        n_ori_in_optical(1) = qs(1);
        n_ori_in_optical(2) = qs(2);

        n_end_in_optical(0) = qs(0) + qs(3);
        n_end_in_optical(1) = qs(1) + qs(4);
        n_end_in_optical(2) = qs(2) + qs(5);

        ContactManager::frameTransform(n_ori_in_optical, n_ori_in_map, optical_to_map);
        ContactManager::frameTransform(n_end_in_optical, n_end_in_map, optical_to_map);

        dp::Vector6d qs_in_map;
        qs_in_map(0) = n_ori_in_map(0);
        qs_in_map(1) = n_ori_in_map(1);
        qs_in_map(2) = n_ori_in_map(2);
        qs_in_map(3) = n_end_in_map(0) - n_ori_in_map(0);
        qs_in_map(4) = n_end_in_map(1) - n_ori_in_map(1);;
        qs_in_map(5) = n_end_in_map(2) - n_ori_in_map(2);

        cm.getInvKin()->setLinkLength(0.8);
        dp::Vector5d q = cm.getInvKin()->qsToQ(qs_in_map);

        ROS_INFO("u: %f, v: %f, q: %f, %f, %f, %f, %f", u_closest, v_closest,
            q(0), q(1), q(2), q(3), q(4));

        // // get the end point in the end_Link frame
        // // the end point in the end_Link frame
        // geometry_msgs::PointStamped end_point_end_frame;
        // end_point_end_frame.header.frame_id = "end_Link";
        // end_point_end_frame.header.stamp = ros::Time::now();
        // end_point_end_frame.point.x = 0;
        // end_point_end_frame.point.y = 0;
        // end_point_end_frame.point.z = 0;
        //
        // // the end point in the optical frame
        // geometry_msgs::PointStamped end_point_optical_frame;
        // end_point_optical_frame.header.frame_id = "depth_optical_frame";
        // end_point_optical_frame.header.stamp = ros::Time::now();
        //
        // // the closest point in the optical frame
        // geometry_msgs::PointStamped closet_point_optical_frame;
        // closet_point_optical_frame.header.frame_id = "depth_optical_frame";
        // closet_point_optical_frame.header.stamp = ros::Time::now();
        //
        // // the closest point in the map frame
        // geometry_msgs::PointStamped closet_point_map_frame;
        // closet_point_map_frame.header.frame_id = "map";
        // closet_point_map_frame.header.stamp = ros::Time::now();
        //
        // // get the end point in the optical frame
        // tf2::doTransform(end_point_end_frame, end_point_optical_frame, end_to_optical);
        //
        // // get the closest point on the surface in the optical frame
        // double u_closest, v_closest;
        // Eigen::Vector3d end_effector_point{end_point_optical_frame.point.x,
        //     end_point_optical_frame.point.y, end_point_optical_frame.point.z};
        // cm.getSurface()->getClosestPoint(end_effector_point, u_closest, v_closest);
        // Eigen::Vector3d closet_point_optical_frame_vec;
        // dp::Vector5d qs = cm.getInvKin()->xToQs(u_closest, v_closest);
        // dp::Vector5d q = cm.getInvKin()->qsToQ(qs);
        // closet_point_optical_frame.point.x = closet_point_optical_frame_vec[0];
        // closet_point_optical_frame.point.y = closet_point_optical_frame_vec[1];
        // closet_point_optical_frame.point.z = closet_point_optical_frame_vec[2];
        //
        // // get the closet point in the map frame
        // tf2::doTransform(closet_point_optical_frame, closet_point_map_frame, end_to_optical);


        // get the corresponding point in the map frame
        visualization_msgs::Marker surface_marker, point_marker, normal_marker;
        surface_marker.header.stamp = ros::Time::now();
        surface_marker.header.frame_id = "depth_optical_frame";

        point_marker.header.stamp = ros::Time::now();
        point_marker.header.frame_id = "depth_optical_frame";

        normal_marker.header.stamp = ros::Time::now();
        normal_marker.header.frame_id = "depth_optical_frame";

        // viz.vizPoint(cm.getPoint()[0], cm.getPoint()[1], point_marker);
        // viz.vizNormal(cm.getPoint()[0], cm.getPoint()[1], normal_marker);
        viz.vizPoint(u_closest, v_closest, point_marker);
        viz.vizNormal(u_closest, v_closest, normal_marker);
        viz.vizSurface(surface_marker);

        pub_surface.publish(surface_marker);
        pub_point.publish(point_marker);
        pub_normal.publish(normal_marker);

        visualization_msgs::Marker base_marker;
        base_marker.header.stamp = ros::Time::now();
        base_marker.header.frame_id = "map";
        base_marker.type = visualization_msgs::Marker::SPHERE;
        base_marker.action = visualization_msgs::Marker::ADD;
        base_marker.pose.position.x = q(0);
        base_marker.pose.position.y = q(1);
        base_marker.pose.position.z = q(2);
        base_marker.pose.orientation.x = 0;
        base_marker.pose.orientation.y = 0;
        base_marker.pose.orientation.z = 0;
        base_marker.pose.orientation.w = 1;
        base_marker.scale.x = 0.1;
        base_marker.scale.y = 0.1;
        base_marker.scale.z = 0.1;

        base_marker.color.r = 1.0;
        base_marker.color.g = 0.0;
        base_marker.color.b = 0.0;
        base_marker.color.a = 1.0;

        pub_base_point.publish(base_marker);

        std_msgs::Float32MultiArray target_msg;
        target_msg.data = {q(0), q(1), q(2), q(3), q(4)};
        target_pub.publish(target_msg);

        loop_rate.sleep();
    }

    return 0;
}