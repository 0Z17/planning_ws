from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import rospkg
import numpy as np
import time
import sys
path = rospkg.RosPack().get_path('planning_utils')
sys.path.append(path + '/src')
from uam import UAM
from scipy.interpolate import interp1d

client = RemoteAPIClient("172.30.144.1")
sim = client.require('sim')

curve_name = 'curve_01'
data_path = path + "/data/" + curve_name + "/"

uam = UAM()
state_traj = np.load(data_path +'state_path_01.npy')
se3_traj = np.load(data_path +'se3_path_01.npy')


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
# all_states = state_traj

# process the time assigment
states_diff = np.abs(np.diff(all_states, axis=0))
max_vel = [0.5,0.5,0.5, np.radians(20), np.radians(5)]
# max_vel = [0.5,0.5,0.5, np.radians(20), np.radians(7)]
# max_vel = [4,4,4, np.radians(0), np.radians(10)]

duration = np.max(states_diff/max_vel, axis=1)
switch_time = np.sum(duration[:count1 + count2])
time_array = np.cumsum(duration)
time_array = np.hstack((0, time_array))

interp_func = interp1d(time_array, all_states, axis=0)

rate = 0.005

uam.set_state(list(initial_state))
count = 0
sim.setNamedBoolParam("record", False)

print("switch time: ", switch_time)
print("task time: ", time_array[-1]-switch_time)
print("time elapsed: ", time_array[-1])

sim.startSimulation()

# open the LiDAR sensor
# open the LiDAR sensor
pos = np.linspace(0,- np.pi/6,50)
for p in pos:
    uam.set_joint_position(p)
    time.sleep(0.01)
sim.setNamedBoolParam("LiDAR", True)
time.sleep(5)
sim.setNamedBoolParam("LiDAR", False)
for p in reversed(pos):
    uam.set_joint_position(p)
    time.sleep(0.01)
time.sleep(3)

while rate*count < time_array[-1]:
    uam.set_state(list(interp_func(rate*count)))
    time.sleep(rate)
    count += 1
    if rate*count > switch_time:
        sim.setNamedBoolParam("record", True)
    # if count == (count1 + count2 + 169):
        # time.sleep(5)

time.sleep(5)
sim.stopSimulation()