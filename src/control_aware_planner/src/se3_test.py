from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import rospkg
import numpy as np
import time
import sys
path = rospkg.RosPack().get_path('planning_utils')
sys.path.append(path + '/src')
from uam import UAM

client = RemoteAPIClient("172.30.144.1")
sim = client.require('sim')

curve_name = 'curve_03'
data_path = path + "/data/" + curve_name + "/"

uam = UAM()
state_traj = np.load(data_path +'state_path.npy')
se3_traj = np.load(data_path +'se3_path.npy')


initial_state = np.array([0,0,2.425,0,0])
# initial_state = np.array([-2.24734,5.775,54.5296,0,0])
# path to the ready position
ready_state = np.hstack((state_traj[0][:3] + se3_traj[0][1]*0.5, state_traj[0][3:]))
# get the transition path

count1 = 100
count2 = 60
trans_path_1 = np.linspace(initial_state, ready_state, count1)
trans_path_2 = np.linspace(ready_state, state_traj[0], count2)

all_states = np.vstack((trans_path_1, trans_path_2, state_traj))

uam.set_state(list(initial_state))
count = 0
sim.setBoolParam(17, False)

sim.startSimulation()

for i in range(all_states.shape[0]):
    uam.set_state(list(all_states[i]))
    time.sleep(0.05)
    count += 1
    if count == (count1 + count2):
        sim.setBoolParam(17, True)
    # if count == (count1 + count2 + 169):
    #     time.sleep(5)

time.sleep(5)

sim.stopSimulation()