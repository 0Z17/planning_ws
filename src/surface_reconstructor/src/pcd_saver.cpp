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

class PCDSaver
{
public:
    PCDSaver(): tf_listener_(tf_buffer_)
    {
        saver_sub = nh_.subscribe("/save_signal", 1, &PCDSaver::save_callback, this);
        pointcloud_sub = nh_.subscribe("/filtered_point_cloud", 1, &PCDSaver::pointcloud_callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber saver_sub;
    ros::Subscriber pointcloud_sub;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::string save_path_{"/home/wsl/proj/planning_ws/src/surface_reconstructor/data"};

    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // Get the frame transform
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "depth_optical_frame", ros::Time(0));
        // geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "camera_depth_optical_frame", ros::Time(0));

        sensor_msgs::PointCloud2 transformed_msg;
        pcl_ros::transformPointCloud("map", *msg, transformed_msg, tf_buffer_);


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_saver");
    PCDSaver pcd_saver;
    ros::spin();
    return 0;
}