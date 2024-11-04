#include <ros/ros.h>
#include <nurbs_class.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

class SurfaceGenerator
{
    public:
        SurfaceGenerator()
        {
            // publish fitted surface
            ROS_INFO("Initializing surface generator...");
            cloud_sub_ = nh_.subscribe("/filtered_point_cloud", 1, &SurfaceGenerator::cloud_callback, this);
            mesh_pub_ = nh_.advertise<visualization_msgs::Marker>("/surface_mesh", 1);
        }
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub_;
        ros::Publisher mesh_pub_;

        // get point cloud and convert to NURBS surface
        void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
        {
            ROS_INFO("Received point cloud message...");
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*cloud_msg, *cloud);

            // Fit NURBS surface
            ROS_INFO("Fitting NURBS surface...");
            surface_reconstructor::Nurbs surface(cloud);
            surface.fitSurface();

            // Initialize mesh marker
            visualization_msgs::Marker mesh_marker;
            mesh_marker.header.frame_id = "map";
            mesh_marker.header.stamp = ros::Time::now();
            mesh_marker.ns = "surface_mesh";
            mesh_marker.id = 0;

            surface.convertToMarker(mesh_marker);

            mesh_pub_.publish(mesh_marker);
        }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_generator");
    SurfaceGenerator surface_generator;

    ROS_INFO("Debugging surface generator...");
    ros::spin();
    return 0;
}

