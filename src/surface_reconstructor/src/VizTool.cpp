#include "VizTool.h"
#include <geometry_msgs/Point.h>

using namespace surface_reconstructor;

VizTool::VizTool(Nurbs* surface): surface_(surface) {}
VizTool::~VizTool()= default;

void VizTool::vizSurface(visualization_msgs::Marker& marker, double lifetime) const {
    // Set the marker's properties
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // Convert the NURBS surface to a triangle mesh
    pcl::PolygonMesh mesh;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(surface_->getSurface(), mesh, 128);

    // Convert PCLPolygonMesh's cloud to pcl::PointCloud<Point>
    const pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    // Add the points to the marker
    marker.points.reserve(mesh.polygons.size() * 3); // Reserve space for the points
    for (const auto& polygon : mesh.polygons)
    {
        for (int i = 0; i < 3; i++)
        {
            const pcl::PointXYZ& point = mesh_cloud->at(polygon.vertices[i]);
            geometry_msgs::Point g_point;
            g_point.x = point.x;
            g_point.y = point.y;
            g_point.z = point.z;
            marker.points.push_back(g_point);
        }
    }

    // Set the marker's color and scale
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f; // Red color with full opacity
    marker.scale.x = 1.0; // Scale doesn't matter for TRIANGLE_LIST
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the marker's lifetime
    marker.lifetime = ros::Duration(lifetime);
}

void VizTool::vizPoint(double u, double v,visualization_msgs::Marker& marker, double lifetime) const {
    // Set the marker's properties
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // Get the point on the surface
    Eigen::Vector3d point;
    surface_->getPos(u, v, point);

    // set the marker's position and orientation
    marker.pose.position.x = point[0];
    marker.pose.position.y = point[1];
    marker.pose.position.z = point[2];
    marker.pose.orientation.w = 1.0;

    // Set the marker's color and scale
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f; // Green color with full opacity

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the marker's lifetime
    marker.lifetime = ros::Duration(lifetime);
}

void VizTool::vizNormal(double u, double v, visualization_msgs::Marker& marker, double lifetime) const {
    // Set the marker's properties
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Get the normal at the point
    Eigen::Vector3d normal;
    surface_->getNormal(u, v, normal);

    // set the start and end points of the arrow
    geometry_msgs::Point start, end;
    Eigen::Vector3d start_point;
    surface_->getPos(u, v, start_point);
    start.x = start_point[0];
    start.y = start_point[1];
    start.z = start_point[2];

    constexpr double scale = 0.3;
    end.x = start_point[0] + normal[0] * scale;
    end.y = start_point[1] + normal[1] * scale;
    end.z = start_point[2] + normal[2] * scale;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // set the length and width of the arrow
    marker.scale.x = 0.02;
    marker.scale.y = 0.04;

    // set the marker's color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    // Set the marker's lifetime
    marker.lifetime = ros::Duration(lifetime);
}



