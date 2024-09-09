import numpy as np
import rospkg 

data_path = rospkg.RosPack().get_path('planning_utils') + '/data/'

def circle_traj(r, t):
    """
    Generate a circular trajectory with radius r and duration t.
    """
    rate = 20
    time_all = t
    x, y = [], []
    
    for i in range(rate * time_all):
        x.append(r * np.cos(2 * np.pi * i / (rate * time_all)))
        y.append(r * np.sin(2 * np.pi * i / (rate * time_all)))

    z = [1.2] * (rate * time_all)
    yaw = [0] * (rate * time_all)
    joint_pos = [0] * (rate * time_all)

    np.save(data_path + 'circle_traj.npy', np.stack([x,y,z,yaw,joint_pos], axis=1))

if __name__ == '__main__':
    circle_traj(0.5, 30)