import numpy as np
import os
from scipy.interpolate import interp1d

class IkSolver:

    def __init__(self, curve=None, link_length=0.947):
        self.curve = curve
        self.link_length = link_length

    def set_link_lengths(self, link_length):
        self.link_length = link_length

    def update_curve(self, curve):
        self.curve = curve

    def uv_to_se3(self, u, v):
        """
        convert the vectors in the uv frame to the se3 frame
        """
        if self.curve is None:
            raise ValueError("No curve is set")
        
        return self.curve.getPoint(u, v)
    
    def se3_to_state(self,position, normal):
        """
        Compute the state of the UAM given the position and normal vectors of the target point
        """
        psi = np.arctan2(-normal[1],-normal[0])
        theta = np.arctan2( - normal[2],np.sqrt(normal[0]**2 + normal[1]**2))
        pos = position + normal*self.link_length
        return np.append(pos, [psi, theta])
    
    def time_alloc(self, val, num = 100, kind = 'cubic'):
        """
        interpolate the value at time step t
        """
        t = np.linspace(0, 1, len(val))
        f = interp1d(t, val, kind=kind)
        t_new = np.linspace(0, 1, num)
        val_new = f(t_new)
        return val_new, f
    
    def get_dpsi(self, n, dn, du, dv):
        """
        convert the vectors in the dzeta frame to the dq in end-effector frame
        """
        dn = dn[0]*du + dn[1]*dv
        dpsi = n[0]/(n[0]**2 + n[1]**2)*dn[1] - n[1]/(n[0]**2 + n[1]**2)*dn[0]
        return dpsi
    
    def get_dtheta(self, n, dn, du, dv):
        """
        convert the vectors in the dzeta frame to the dq in end-effector frame
        """
        n_ = np.sqrt(n[0]**2 + n[1]**2)
        dn = dn[0]*du + dn[1]*dv
        dtheta = - n_/(n[0]**2 + n[1]**2 + n[2]**2)*dn[2] \
                + n[2]/(n[0]**2 + n[1]**2 + n[2]**2) \
                * (n[0]/n_*dn[0] + n[1]/n_*dn[1])
                
        return dtheta
    
    def get_jaccobian(self, theta, l):
        """
        compute the jacobian matrix of the end-effector frame w.r.t. thestate
        """
        jac = np.array([[np.cos(theta), 0, -np.sin(theta), 0, 0],
                        [ 0, 1, 0,  l*np.cos(theta), 0],
                        [np.sin(theta), 0,  np.cos(theta), 0, l],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1]])
        return jac
    
    def zeta_to_vse3(self,u,v,du, dv):
        """
        convert the vectors in the zeta frame to the se3 frame
        """
        deriv = self.curve.getPointDeriv(u, v)
        n = self.curve.getPointNormal(u,v)
        dn = self.curve.getPointNormalDeriv(u,v)
        tu =np.array(deriv[1][0])
        tv = np.array(deriv[0][1])
        dp = tu*du + tv*dv
        dpsi = self.get_dpsi(n, dn, du, dv)
        dtheta = self.get_dtheta(n, dn, du, dv)

        return np.append(dp, [dpsi, dtheta])


if __name__ == '__main__':

    base_path = os.path.dirname(__file__) + '/../../../'

    ik_solver = IkSolver()
    ik_solver.set_link_lengths(0.95)

    traj = np.load(base_path + 'data/uv_traj.npy')

    pos_u = traj[:, 0, 0]
    pos_v = traj[:, 0, 1]
    pos = ik_solver.uv_to_se3(pos_u, pos_v)

    vel_u = traj[:, 1, 0]
    vel_v = traj[:, 1, 1]
    vel = ik_solver.uv_to_se3(vel_u, vel_v)

    np.save(os.path.join(base_path, 'data/state_traj_pos.npy'), pos)
    print('path saved!')