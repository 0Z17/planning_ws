#include "invkin.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
namespace sr = surface_reconstructor;
namespace dp = dynamic_planning;

void ReadPcdFile(std::string pcd_file, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
}

int main() {
    // load the point cloud
    std::string pcd_file = "/home/wsl/proj/pcl/test/milk.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ReadPcdFile(pcd_file, cloud);

    // fit the surface
    sr::Nurbs nurbs(cloud);
    nurbs.fitSurface();

    // extract the surface info

    // get the position of the surface
    dp::InvKin invkin(&nurbs);
    dp::Vector6d qs = invkin.xToQs(0.2, 0.3);
    std::cout << "qs: " << qs << std::endl;

    // get the normal of the surface
    Eigen::Vector3d normal;
    nurbs.getNormal(0.2, 0.3, normal);
    std::cout << "normal: " << normal << std::endl;

    // get the derivative of the normal
    Eigen::Vector3d dn_u, dn_v;
    nurbs.getDNormal(0.2, 0.3, dn_u, dn_v);
    std::cout << "dn_u: " << dn_u << std::endl;
    std::cout << "dn_v: " << dn_v << std::endl;

    // get the config of the end effector
    dp::Vector5d qe = dp::InvKin::qsToQe(qs);
    std::cout << "qe: " << qe << std::endl;

    // get the change of position of the end effector
    Eigen::Vector3d dps_u, dps_v;
    invkin.getDpos(0.2, 0.3, dps_u, dps_v);
    std::cout << "dps_u: " << dps_u << std::endl;
    std::cout << "dps_v: " << dps_v << std::endl;

    // get the change of psi
    const double dpsi = dp::InvKin::getDpsi(normal, dn_u, dn_v, 0.1, 0.1);
    std::cout << "dpsi: " << dpsi << std::endl;

    // get the change of theta
    const double dtheta = dp::InvKin::getDtheta(normal, dn_u, dn_v, 0.1, 0.1);
    std::cout << "dtheta: " << dtheta << std::endl;

    // get the change of the end effector config
    const dp::Vector5d dqe = invkin.dxToDqe(0.2, 0.3, 0.1, 0.1);
    std::cout << "dqe: " << dqe << std::endl;

}
