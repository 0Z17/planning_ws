#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/surface/3rdparty/opennurbs/opennurbs.h"

#ifndef NURBS_CLASS_H_
#define NURBS_CLASS_H_

namespace surface_reconstructor {

class Nurbs
{
public:
    Nurbs(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    ~Nurbs();

    int getPos(double u, double v, ON_3dPoint& pos);
   
    int getNormal(double u, double v, ON_3dVector& normal);

    int get1Deriv(double u, double v, ON_3dVector& du, ON_3dVector& dv);
    
    int get2Deriv(double u, double v, ON_3dVector& du, ON_3dVector& dv, ON_3dVector& duu, ON_3dVector& duv, ON_3dVector& dvv);

    int getCurvature(double u, double v, double& curvature);

    int setFittingParams(
        double interior_smoothness = 0.1,
        double interior_weight = 1.0,
        double boundary_smoothness = 0.1,
        double boundary_weight = 0.0
    );

    int fitSurface();

    int convertToMarker(visualization_msgs::Marker& marker);

private:
    bool is_fitted_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    ON_NurbsSurface surface_;
    pcl::on_nurbs::FittingSurface::Parameter params_;
    unsigned refinement_ = 2;
    unsigned iterations_ = 10;
};

}
#endif // NURBS_CLASS_H_