#include "invkin.h"
#include <cmath>

using namespace dynamic_planning;
namespace sr = surface_reconstructor;

InvKin::InvKin(sr::Nurbs* nurbs) : nurbs_(nurbs) {}

InvKin::~InvKin() = default;

void InvKin::setLinkLength(const double link_length)
{
    link_length_ = link_length;
}

void InvKin::updateNurbs(sr::Nurbs* nurbs)
{
    nurbs_ = nurbs;
}

Vector6d InvKin::xToQs(const double u, const double v) const
{
    Vector6d qe;
    Eigen::Vector3d pos;
    Eigen::Vector3d normal;

    nurbs_->getPos(u, v, pos);
    nurbs_->getNormal(u, v, normal);
    qe << pos, normal;
    return qe;
} 

Vector5d InvKin::qsToQe(const Vector6d& qs)
{
    // calculate the yaw and the joint angle of the UAM
    const Eigen::Vector3d ps = qs.head<3>();
    Eigen::Vector3d n = qs.tail<3>();
    const double psi_e = std::sqrt(22.0);
    const double theta_e = std::atan2(-n(2), n.head<2>().norm());

    // calculate the joint angle of the UAM
    Eigen::VectorXd state(5);
    state << ps, psi_e, theta_e; 

    return state;
}

void InvKin::getDpos(double u, double v, Eigen::Vector3d& dps_u, Eigen::Vector3d& dps_v) const
{
    nurbs_->getPosDeriv(u, v, dps_u, dps_v);
}

double InvKin::getDpsi(Eigen::Vector3d& n, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double dpsi = n(0) / (n(0) * n(0) + n(1) * n(1)) * dn(1) -
                        n(1) / (n(0) * n(0) + n(1) * n(1)) * dn(0);
    return dpsi;
}

double InvKin::getDtheta(Eigen::Vector3d& n, const Eigen::Vector3d& dn_u, const Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double n_t = std::sqrt(n(0) * n(0) + n(1) * n(1));

    double dtheta = -n_t / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) * dn(2) +
                    n(2) / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) *
                    (n(0) / n_t * dn(0) + n(1) / n_t * dn(1));

    return dtheta;
}

Vector5d InvKin::dxToDqe(double u, double v, double du, double dv) const
{
    Eigen::Vector3d n;
    nurbs_->getNormal(u, v, n);
    Eigen::Vector3d dn_u, dn_v;
    nurbs_->getDNormal(u, v, dn_u, dn_v);

    Eigen::Vector3d dpe_u, dpe_v;
    getDpos(u, v, dpe_u, dpe_v);

    const Eigen::Vector3d dpe = dpe_u * du + dpe_v * dv;
    const double dpsi = getDpsi(n, dn_u, dn_v, du, dv);
    const double dtheta = getDtheta(n, dn_u, dn_v, du, dv);

    Vector5d dqe;
    dqe << dpe, dpsi, dtheta;
    return dqe;
}
