#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/bilateral.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "nurbs_class.h"
// #include <pcl_ros/point_cloud.h>

class PointCloudProcessor
{
public:
    PointCloudProcessor()
    {
        // subscribe to point cloud topic
        point_cloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1, &PointCloudProcessor::pointCloudCallback, this);
        // publish filtered point cloud topic
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
        compare_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud_compare", 1);
        mesh_pub_ = nh_.advertise<visualization_msgs::Marker>("/surface_mesh", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher compare_point_cloud_pub_;
    ros::Publisher mesh_pub_;

    // get point cloud data and process it
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) const {
        // convert ROS PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*input_cloud_msg, *cloud);

        // screen out points on the operator 
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::PassThrough<pcl::PointXYZRGB> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("z");  // 设置要过滤的轴，例如 z 轴
        // pass.setFilterLimits(0.0, 1.0);  // 设置 z 轴的值范围
        // pass.filter(*cloud_filtered);  // 执行过滤

        // Crop box filter
        // pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
        // crop_box_filter.setInputCloud(cloud);
        // // ROS_INFO("Before crop box filter: %d points", cloud->size());
        // // Set the box dimensions in the camera coordinate frame (adjust these values based on your application)
        // crop_box_filter.setMin(Eigen::Vector4f(-0.7, -0.8, -1.5, 1.0)); // Example limits for X, Y, Z
        // crop_box_filter.setMax(Eigen::Vector4f(0.6, 0.65, 1.9, 1.0));
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZRGB>());
        // crop_box_filter.filter(*cloud_cropped);
        // ROS_INFO("Cropped cloud: %d points", cloud_cropped->size());


        // // Apply moving least squares filter
        // pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        // mls.setInputCloud(cloud); // 输入点云
        // mls.setSearchRadius(0.03); // 设置搜索半径
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // mls.process(*smoothed_cloud); // 输出点云


        // filter out points with low density
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        // sor.setInputCloud(filtered_cloud);
        // sor.setInputCloud(cloud_cropped);
        double leaf_size = 0.03;  // the size of the voxels in the grid
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);  // set the size of the voxels
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
        sor.filter(*cloud_downsampled);  // filter the cloud

        // Apply statistical outlier removal filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_outlier;
        sor_outlier.setInputCloud(cloud_downsampled);
        // sor_outlier.setInputCloud(cloud_cropped);
        sor_outlier.setMeanK(20);  // Set the number of nearest neighbors to use for mean distance estimation
        sor_outlier.setStddevMulThresh(1.0);  // Set the standard deviation multiplier for the distance threshold
        sor_outlier.filter(*cloud_filtered);  // filter the outliers

        // convert PCL point cloud to ROS PointCloud2 message
        sensor_msgs::PointCloud2 output_cloud_msg;
        sensor_msgs::PointCloud2 compare_cloud_msg;
        pcl::toROSMsg(*cloud_filtered, output_cloud_msg);
        // pcl::toROSMsg(*cloud_downsampled, output_cloud_msg);
        pcl::toROSMsg(*cloud, compare_cloud_msg);
        // pcl::toROSMsg(*cloud_filtered, output_cloud_msg);
        output_cloud_msg.header = input_cloud_msg->header;
        compare_cloud_msg.header = input_cloud_msg->header;

        // publish filtered point cloud
        point_cloud_pub_.publish(output_cloud_msg);
        compare_point_cloud_pub_.publish(compare_cloud_msg);


        // Fit NURBS surface
        // ROS_INFO("Fitting NURBS surface...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*cloud_filtered, *cloud_output);
        surface_reconstructor::Nurbs surface(cloud_output);
        // surface_reconstructor::Nurbs surface(cloud_downsampled);
        surface.fitSurface();

        // Initialize mesh marker
        visualization_msgs::Marker mesh_marker;
        // mesh_marker.header.frame_id = "map";
        // mesh_marker.header.stamp = ros::Time::now();
        mesh_marker.header = input_cloud_msg->header;
        mesh_marker.ns = "surface_mesh";
        mesh_marker.id = 0;

        surface.convertToMarker(mesh_marker);
        mesh_pub_.publish(mesh_marker);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processor");
    PointCloudProcessor pc_processor;

    ros::spin();  // keep node running until shut down
    return 0;
}
