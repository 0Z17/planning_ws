import numpy as np
import os
from scipy.interpolate import interp1d

class IkSolver:

    def __init__(self):
        self.link_length = None
        self.curve = None

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
        theta = np.arctan2(normal[1],normal[0])
        phi = -np.arctan2(normal[2],np.sqrt(normal[0]**2 + normal[1]**2))
        pos = position + normal*self.link_len
        return pos, theta, phi
    
    def time_alloc(self, val, num = 100, kind = 'cubic'):
        """
        interpolate the value at time step t
        """
        t = np.linspace(0, 1, len(val))
        f = interp1d(t, val, kind=kind)
        t_new = np.linspace(0, 1, num)
        val_new = f(t_new)
        return val_new, f

if __name__ == '__main__':

    base_path = os.path.dirname(__file__) + '/../../../'

    ik_solver = IkSolver()
    ik_solver.set_link_lengths(0.5)

    traj = np.load(base_path + 'data/uv_traj.npy')

    pos_u = traj[:, 0, 0]
    pos_v = traj[:, 0, 1]
    pos = ik_solver.uv_to_se3(pos_u, pos_v)

    vel_u = traj[:, 1, 0]
    vel_v = traj[:, 1, 1]
    vel = ik_solver.uv_to_se3(vel_u, vel_v)

    np.save(os.path.join(base_path, 'data/state_traj_pos.npy'), pos)
    print('path saved!')