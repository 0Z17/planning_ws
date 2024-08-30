from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import rospkg
import numpy as np
import time
import sys

rospack = rospkg.RosPack()
path = rospack.get_path('planning_utils')
sys.path.append(path + '/src')

from uam import UAM

client = RemoteAPIClient()
sim = client.require('sim')

dummy_handle = sim.getObject('/Dummy')

uam = UAM()
state_traj = np.load(path + '/data/state_traj.npy')
se3_traj = np.load(path + '/data/se3_traj.npy')

# for i in range(se3_traj.shape[0]):
#     sim.setObjectPosition(dummy_handle, list(se3_traj[i][0]))
#     time.sleep(0.1)

# for i in range(state_traj.shape[0]):
#     uam.set_state(list(state_traj[i]))
#     time.sleep(0.2)