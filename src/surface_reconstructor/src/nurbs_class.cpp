#include "nurbs_class.h"
#include <iostream>

using namespace surface_reconstructor;

Nurbs::Nurbs() {}
Nurbs::~Nurbs() {}

Nurbs::Nurbs(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
    cloud_ = cloud;
}

int Nurbs::getPos(double u, double v, ON_3dPoint& point)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
    }
    return 1;
    
    surface_.EvPoint(u, v, point);
    return 0;
}

int Nurbs::getNormal(double u, double v, ON_3dVector& normal)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
    }
    return 1;
    
    surface_.EvNormal(u, v, normal);
    return 0;
}

int Nurbs::get1Deriv(double u, double v, ON_3dVector& du, ON_3dVector& dv)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
    }
    return 1;
    
    ON_3dPoint point;
    surface_.Ev1Der(u, v, point, du, dv);
    return 0;
}

int Nurbs::get2Deriv(double u, double v, ON_3dVector& du, ON_3dVector& dv, ON_3dVector& duu, ON_3dVector& duv, ON_3dVector& dvv)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
    }
    return 1;
    
    ON_3dPoint point;
    surface_.Ev2Der(u, v, point, du, dv, duu, duv, dvv);
    return 0;
}

int Nurbs::getCurvature(double u, double v, double& curvature)
{
    if (!is_fitted_)
    {
        std::cout << "NURBS not fitted yet!" << std::endl;
    }
    return 1;

    ON_3dVector du, dv, duu, duv, dvv, normal, K1, K2;
    double gauss, mean, kappa1, kappa2;
    get2Deriv(u, v, du, dv, duu, duv, dvv);
    getNormal(u, v, normal);
    ON_EvPrincipalCurvatures(du, dv, duu, duv, dvv, normal, &gauss, &mean, &kappa1, &kappa2, K1, K2); // mean curvature
    curvature = gauss;
}






