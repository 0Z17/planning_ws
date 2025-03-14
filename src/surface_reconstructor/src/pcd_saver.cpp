#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "nurbs_class.h"
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class PCDSaver
{
public:
    PCDSaver(): tf_listener_(tf_buffer_)
    {
        publish_static_transform();
        saver_sub = nh_.subscribe("/save_signal", 1, &PCDSaver::save_callback, this);
        pointcloud_sub = nh_.subscribe("/filtered_point_cloud", 1, &PCDSaver::pointcloud_callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber saver_sub;
    ros::Subscriber pointcloud_sub;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    std::string save_path_{"/home/wsl/proj/planning_ws/src/surface_reconstructor/data"};

    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // Get the frame transform
        // geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "depth_optical_frame", ros::Time(0));
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("floor", "depth_optical_frame", ros::Time(0));
        // geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "camera_depth_optical_frame", ros::Time(0));

        sensor_msgs::PointCloud2 transformed_msg;
        // pcl_ros::transformPointCloud("map", *msg, transformed_msg, tf_buffer_);
        pcl_ros::transformPointCloud("floor", *msg, transformed_msg, tf_buffer_);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(transformed_msg, *cloud_temp);
        cloud_ = cloud_temp;
    }

    void save_callback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data == true && cloud_!= nullptr)
        {
            ROS_INFO("Saving pointcloud");
            // Save the pointcloud to a file
            pcl::PCDWriter writer;
            writer.write(save_path_ + "/pointcloud.pcd", *cloud_, false);
            ROS_INFO("Pointcloud saved");
        }
    }


    void publish_static_transform()
    {
        geometry_msgs::TransformStamped static_transform;
        static_transform.header.stamp = ros::Time::now();
        // static_transform.header.frame_id = "map";   // parent frame
        // static_transform.child_frame_id = "base_link";      // child frame
        static_transform.header.frame_id = "floor";   // parent frame
        static_transform.child_frame_id = "map";      // child frame
        
        // set the translation (x=0.03, y=0, z=0.34)
        static_transform.transform.translation.x = 0.03;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.34;

        // static_transform.transform.translation.x = 1.1796;
        // static_transform.transform.translation.y = 1.1948;
        // static_transform.transform.translation.z = 1.3743;
        
        // set the rotation (roll=pitch=yaw=0)
        tf2::Quaternion quat;
        // quat.setRPY(0, 0, 3.141592653589); 
        quat.setRPY(0, 0, 0); 
        static_transform.transform.rotation.x = quat.x();
        static_transform.transform.rotation.y = quat.y();
        static_transform.transform.rotation.z = quat.z();
        static_transform.transform.rotation.w = quat.w();
        
        // publish the static transform
        static_broadcaster_.sendTransform(static_transform);
        ROS_INFO("Static transform published: floor -> map");
        // ROS_INFO("Static transform published: map -> base_link");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_saver");
    PCDSaver pcd_saver;
    ros::spin();
    return 0;
}