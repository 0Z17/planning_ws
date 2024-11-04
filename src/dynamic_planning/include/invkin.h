#ifndef DYNAMIC_PLANNING_INVKIN_H
#define DYNAMIC_PLANNING_INVKIN_H

#include "nurbs_class.h"
#include <Eigen/Dense>


namespace dynamic_planning {

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
namespace sr = surface_reconstructor;

class InvKin {
public:
    explicit InvKin(sr::Nurbs* nurbs);
    ~InvKin();

    /**
     * @brief set the link length of the manipulator
     * @param link_length the length of the manipulator's links
     */
    void setLinkLength(double link_length);

    /**
     * @brief Updates the NURBS (Non-Uniform Rational B-Splines) object.
     * 
     * This function takes a reference to a NurbsClass object and updates its state.
     * 
     * @param nurbs Reference to a NurbsClass object that will be updated.
     */
    void updateNurbs(sr::Nurbs* nurbs);

    
    /**
     * @brief Converts the surface params [x = (u,v)] to surface config [q_s = (p_s,n)].
     * @param u The first parameter for the surface.
     * @param v The second parameter of the surface.
     * @return The corresponding point on the surface [q_s], consisting of the contact 
     * point and the normal vector.
     */
    Vector6d xToQs(double u, double v) const;

    /**
     * @brief Converts the surface config [q_s] to the end effector config [q_e].
     * @param qs The surface config [q_s], consisting of the contact point and the normal vector.
     * @return The config of the end effector [q_e], consisting of the position[p_e = (x_e,y_e,z_e)],
     *         yaw[psi_e] and pitch[theta_e].
     */
    static Vector5d qsToQe(const Vector6d& qs) ;

    /**
     * @brief get the change of position of the end effector [dp_e] respect to the 
     *        surface params [(u,v)] with the change of surface params [(du,dv)].
     * @param u The first parameter for the surface.
     * @param v The second parameter of the surface.
     * @param du The change of u.
     * @param dv The change of v.
     * @return The change of the position of the end effector [dp_e].
     */
    void getDpos(double u, double v, Eigen::Vector3d& dps_u, Eigen::Vector3d& dps_v) const;

    /**
     * @brief get the change of the yaw of the end effector [dpsi_e] respect to 
     *        the surface params [(u,v)] with the change of surface params [(du,dv)].
     * @param u The first parameter for the surface.
     * @param v The second parameter of the surface.
     * @param du The change of u.
     * @param dv The change of v.
     * @return The change of the yaw of the end effector [dpsi_e].
     */
    static double getDpsi(Eigen::Vector3d& n, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v,
                          double du, double dv);

    /**
     * @brief get the change of the joint angle of the manipulator [dtheta_e]
     *         respect to the surface params [(u,v)] with the change of surface params [(du,dv)].
     * @param u The first parameter for the surface.
     * @param v The second parameter of the surface.
     * @param du The change of u.
     * @param dv The change of v.
     * @return The change of the joint angle of the manipulator [dtheta_e].
     */
    static double getDtheta(Eigen::Vector3d& n, const Eigen::Vector3d& dn_u, const Eigen::Vector3d& dn_v,
                     double du, double dv) ;
    
    /**
     * @brief Convert the change of the surface params [(du,dv)] to the change of the end effector config [dq_e].
     * @param u The first parameter for the surface.
     * @param v The second parameter of the surface.
     * @param du The change of u.
     * @param dv The change of v.
     * @return The change of the end effector config [dq_e].
     */
    Vector5d dxToDqe(double u, double v, double du, double dv) const;

private:
    sr::Nurbs* nurbs_;
    double link_length_ = 0.95;
};

} // namespace dynamic_planning

#endif // DYNAMIC_PLANNING_INVKIN_H