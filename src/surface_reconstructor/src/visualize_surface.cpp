#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

using Point = pcl::PointXYZ;

void CreateCylinderPoints(pcl::PointCloud<Point>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data, unsigned npoints,
                          double alpha, double h, double r)
{
    for (unsigned i = 0; i < npoints; i++)
    {
        double da = alpha * double(rand()) / RAND_MAX;
        double dh = h * (double(rand()) / RAND_MAX - 0.5);

        Point p;
        p.x = float(r * std::cos(da));
        p.y = float(r * sin(da));
        p.z = float(dh);

        data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
        cloud->push_back(p);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nurbs_visualization");
    ros::NodeHandle nh;

    // Point Cloud Publisher
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_cylinder", 1);
    // Mesh Publisher (if you want to visualize the mesh in RViz)
    ros::Publisher mesh_pub = nh.advertise<visualization_msgs::Marker>("nurbs_mesh", 1);

    unsigned npoints(200);
    unsigned refinement(2);
    unsigned iterations(10);

    // Create point cloud
    pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
    pcl::on_nurbs::NurbsDataSurface data;
    CreateCylinderPoints(cloud, data.interior, npoints, M_PI, 1.0, 0.5);

    // Convert point cloud to ROS message
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map"; // Set the frame_id

    // Fit NURBS surface
    ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(3, &data);
    pcl::on_nurbs::FittingSurface fit(&data, nurbs);

    pcl::on_nurbs::FittingSurface::Parameter params;
    params.interior_smoothness = 0.1;
    params.interior_weight = 1.0;
    params.boundary_smoothness = 0.1;
    params.boundary_weight = 0.0;

    // NURBS refinement
    for (unsigned i = 0; i < refinement; i++)
    {
        fit.refine(0);
        fit.refine(1);
    }

    // Fitting iterations
    for (unsigned i = 0; i < iterations; i++)
    {
        fit.assemble(params);
        fit.solve();
    }

    // Triangulate NURBS surface
    nurbs = fit.m_nurbs;

    // Try to get the normal vectors for the surface
    ON_3dPoint point;
    ON_3dVector norm;

    double pointAndTangent[15];

    nurbs.EvNormal(0.5, 0.2, point, norm);
    nurbs.Evaluate(0.5, 0.2, 2, 5, pointAndTangent);
    // print all 15 values of pointAndTangent
    for (int i = 0; i < 15; i++)
    {
        std::cout << pointAndTangent[i] << " ";
    }
    std::cout << std::endl;

    // print the normal vector
    std::cout << "Normal vector: " << norm.x << " " << norm.y << " " << norm.z << std::endl;

    pcl::PolygonMesh mesh;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(nurbs, mesh, 128);

    // Convert PCLPolygonMesh's cloud to pcl::PointCloud<Point>
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    // Create a Marker for the mesh
    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "map"; // Set the frame_id
    mesh_marker.header.stamp = ros::Time::now();
    mesh_marker.ns = "nurbs_mesh";
    mesh_marker.id = 0;
    mesh_marker.type = visualization_msgs::Marker::TRIANGLE_LIST; // Define the marker as a triangle list
    mesh_marker.action = visualization_msgs::Marker::ADD;

    // Populate mesh_marker with vertices and triangles from the mesh
    size_t num_vertices = mesh.cloud.width * mesh.cloud.height;
    size_t num_triangles = mesh.polygons.size();

    // Reserve space for points (vertices) and colors (if any)
    mesh_marker.points.reserve(num_vertices * 3); // Each triangle has 3 vertices

    for (const auto& polygon : mesh.polygons)
    {
        for (int i = 0; i < 3; ++i)
        {
            const pcl::PointXYZ& p = mesh_cloud->at(polygon.vertices[i]);
            geometry_msgs::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            mesh_marker.points.push_back(pt);
        }
    }

    // Optionally, set color and scale if needed
    mesh_marker.color.r = 1.0f;
    mesh_marker.color.g = 0.0f;
    mesh_marker.color.b = 0.0f;
    mesh_marker.color.a = 1.0f; // Red color with full opacity
    mesh_marker.scale.x = 1.0; // Scale doesn't matter for TRIANGLE_LIST
    mesh_marker.scale.y = 1.0;
    mesh_marker.scale.z = 1.0;

    // print the cloud_msg data
    // std::cout << "Point Cloud: " << std::endl;
    // for (int i = 0; i < cloud_msg.width * cloud_msg.height; i++)
    // {
    //     std::cout << "x: " << cloud_msg.data[i * cloud_msg.point_step + 0] << " y: "
    //               << cloud_msg.data[i * cloud_msg.point_step + 4] << " z: "
    //               << cloud_msg.data[i * cloud_msg.point_step + 8] << std::endl;
    // }

    // Publish the point cloud
    while (ros::ok())
    {
        point_cloud_pub.publish(cloud_msg);
        mesh_pub.publish(mesh_marker);
        ros::spinOnce();
    }

    // point_cloud_pub.publish(cloud_msg);
    // ros::spin();

    return 0;
}
