#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <nurbs_class.h>
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
        mesh_pub_ = nh_.advertise<visualization_msgs::Marker>("/surface_mesh", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher mesh_pub_;

    // get point cloud data and process it
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg)
    {
        // convert ROS PointCloud2 message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*input_cloud_msg, *cloud);

        // screen out points on the operator 
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pass.setInputCloud(cloud);
        // pass.setFilterFieldName("z");  // 设置要过滤的轴，例如 z 轴
        // pass.setFilterLimits(0.0, 1.0);  // 设置 z 轴的值范围
        // pass.filter(*cloud_filtered);  // 执行过滤

        // filter out points with low density
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);  // set the size of the voxels
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        sor.filter(*cloud_downsampled);  // filter the cloud

        // convert PCL point cloud to ROS PointCloud2 message
        sensor_msgs::PointCloud2 output_cloud_msg;
        pcl::toROSMsg(*cloud_downsampled, output_cloud_msg);
        output_cloud_msg.header = input_cloud_msg->header;

        // publish filtered point cloud
        point_cloud_pub_.publish(output_cloud_msg);

        // Fit NURBS surface
        ROS_INFO("Fitting NURBS surface...");
        surface_reconstructor::Nurbs surface(cloud_downsampled);
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
