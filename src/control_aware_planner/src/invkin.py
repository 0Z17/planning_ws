import numpy as np
import os

class IkSolver:

    def __init__(self):
        self.link_length = None

    def set_link_lengths(self, link_length):
        self.link_length = link_length

    ## only consider the flat plane for now
    def uv_to_se3(self, u, v):
        """
        convert the vectors in the uv frame to the se3 frame
        """
        vx = np.array(u)
        vy = np.array(v)
        vz = np.full(len(vx), 0.0)

        return np.stack((vx, vy, vz), axis=1)
    
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
        f = np.interp1d(t, val, kind=kind)
        t_new = np.linspace(0, 1, num)
        val_new = f(t_new)
        return val_new, f

if __name__ == '__main__':

    base_path = os.path.dirname(__file__) + '/../../../'

    ik_solver = IkSolver()
    ik_solver.set_link_lengths(0.5)

    traj = np.load(base_path + 'data/uv_traj.npy')

    vel_u = traj[:, 1, 0]
    vel_v = traj[:, 1, 1]

    vel = ik_solver.uv2xyz(vel_u, vel_v)

    np.save(os.path.join(base_path, 'data/state_traj.npy'), vel)