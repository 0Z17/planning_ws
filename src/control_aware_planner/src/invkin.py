import numpy as np
import os

class IkSolver:

    def __init__(self):
        self.link_length = None

    def set_link_lengths(self, link_length):
        self.link_length = link_length

    ## only consider the flat plane for now
    def uv2xyz(self, u, v):
        vx = np.array(u)
        vy = np.array(v)
        vz = np.full(len(vx), 0.0)

        return np.stack((vx, vy, vz), axis=1)
    
    
if __name__ == '__main__':
    ik_solver = IkSolver()
    ik_solver.set_link_lengths(0.5)

    traj = np.load(os.path.join(os.path.dirname(__file__), '../input/rrt_path.npy'))

    vel_u = traj[:, 1, 0]
    vel_v = traj[:, 1, 1]

    vel = ik_solver.uv2xyz(vel_u, vel_v)

    np.save(os.path.join(os.path.dirname(__file__), '../output/rrt_vel.npy'), vel)