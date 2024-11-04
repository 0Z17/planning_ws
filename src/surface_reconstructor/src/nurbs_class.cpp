#include "nurbs_class.h"
#include <iostream>

using namespace surface_reconstructor;

Nurbs::Nurbs(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_ = cloud;
    Nurbs::setFittingParams(0.1, 1.0, 0.1, 0.0); // Default parameters
}

Nurbs::~Nurbs() = default;

int Nurbs::getPos(double u, double v, ON_3dPoint& pos)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return 1;
    }

    surface_.EvPoint(u, v, pos);
    return 0;
}

int Nurbs::getPos(double u, double v, Eigen::Vector3d& point)
{
    ON_3dPoint pos;
    getPos(u, v, pos);
    point << pos.x, pos.y, pos.z;
    return 0;
}

int Nurbs::getNormal(double u, double v, ON_3dVector& n) const
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return 1;
    }
    
    surface_.EvNormal(u, v, n);
    return 0;
}

int Nurbs::getNormal(double u, double v, Eigen::Vector3d& normal) const
{
    ON_3dVector n;
    getNormal(u, v, n);
    normal << n.x, n.y, n.z;
    return 0;
}

int Nurbs::getPosDeriv(double u, double v, ON_3dVector& du,
    ON_3dVector& dv) const
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return 1;
    }

    ON_3dPoint point;
    surface_.Ev1Der(u, v, point, du, dv);
    return 0;
}

int Nurbs::getPosDeriv(double u, double v, Eigen::Vector3d& du,
    Eigen::Vector3d& dv) const
{
    ON_3dVector du_t, dv_t;
    getPosDeriv(u, v, du_t, dv_t);
    du << du_t.x, du_t.y, du_t.z;
    dv << dv_t.x, dv_t.y, dv_t.z;
    return 0;
}

int Nurbs::getPos2Deriv(double u, double v, ON_3dVector& du, ON_3dVector& dv,
    ON_3dVector& duu, ON_3dVector& duv, ON_3dVector& dvv)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return 1;
    }

    ON_3dPoint point;
    surface_.Ev2Der(u, v, point, du, dv, duu, duv, dvv);
    return 0;
}

int Nurbs::getPos2Deriv(double u, double v, Eigen::Vector3d& du, Eigen::Vector3d& dv,
    Eigen::Vector3d& duu, Eigen::Vector3d& duv, Eigen::Vector3d& dvv)
{
    ON_3dVector du_t, dv_t, duu_t, duv_t, dvv_t;
    getPos2Deriv(u, v, du_t, dv_t, duu_t, duv_t, dvv_t);
    du << du_t.x, du_t.y, du_t.z;
    dv << dv_t.x, dv_t.y, dv_t.z;
    duu << duu_t.x, duu_t.y, duu_t.z;
    duv << duv_t.x, duv_t.y, duv_t.z;
    dvv << dvv_t.x, dvv_t.y, dvv_t.z;
    return 0;
}

int Nurbs::getCurvature(double u, double v, double& curvature)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
        return 1;
    }

    ON_3dVector du, dv, duu, duv, dvv, normal, K1, K2;
    double gauss, mean, kappa1, kappa2;
    getPos2Deriv(u, v, du, dv, duu, duv, dvv);
    getNormal(u, v, normal);
    ON_EvPrincipalCurvatures(du, dv, duu, duv, dvv, normal, &gauss, &mean, &kappa1, &kappa2, K1, K2); // mean curvature
    curvature = gauss;

    return 0;
}

int Nurbs::setFittingParams(double interior_smoothness, double interior_weight, 
                            double boundary_smoothness, double boundary_weight)
{
    params_.interior_smoothness = interior_smoothness;
    params_.interior_weight = interior_weight;
    params_.boundary_smoothness = boundary_smoothness;
    params_.boundary_weight = boundary_weight;
    return 0;
}

int Nurbs::getDNormal(double u, double v, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v)
{
    Eigen::Vector3d n;
    getNormal(u, v, n);
    Eigen::Vector3d du, dv, duu, duv, dvv;
    getPos2Deriv(u, v, du, dv, duu, duv, dvv);

    // Compute the first fundamental form the NURBS
    double E = du.dot(du);
    double F = du.dot(dv);
    double G = dv.dot(dv);

    // Compute the second fundamental form the NURBS
    double e = duu.dot(n);
    double f = duv.dot(n);
    double g = dvv.dot(n);

    dn_u = (f*F-e*G)/(E*G-F*F)*du + (e*F-f*E)/(E*G-F*F)*dv;
    dn_v = (g*F-f*G)/(E*G-F*F)*du + (f*F-g*E)/(E*G-F*F)*dv;

    return 0;
}

int Nurbs::fitSurface()
{   
    // Initialize NURBS surface
    pcl::on_nurbs::NurbsDataSurface data;
    // Convert cloud to data
    for (const auto& point : cloud_->points)
    {
        data.interior.emplace_back(point.x, point.y, point.z);
    }
    surface_ = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(3, &data, Eigen::Vector3d(-1, 0, 0));
    pcl::on_nurbs::FittingSurface fit (&data, surface_);

    // NURBS refinement
    for (unsigned i = 0; i < refinement_; i++)
    {
        fit.refine(0);
        fit.refine(1);
    }

    // Fitting iteraions
    for (unsigned i = 0; i < iterations_; i++)
    {
        fit.assemble(params_); // Assemble system of equations
        fit.solve();
    }

    surface_ = fit.m_nurbs;

    is_fitted_ = true;
    return 0;
}

int Nurbs::convertToMarker(visualization_msgs::Marker& marker)
{
    // Reset the marker's properties
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // Convert the NURBS surface to a triangle mesh
    pcl::PolygonMesh mesh;
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(surface_, mesh, 128);

    // Convert PCLPolygonMesh's cloud to pcl::PointCloud<Point>
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f; // Red color with full opacity
    marker.scale.x = 1.0; // Scale doesn't matter for TRIANGLE_LIST
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    
    return 0;
}



