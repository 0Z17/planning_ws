import rospkg
import sys
from scipy.io import loadmat, savemat
from invkin import IkSolver
sys.path.append(rospkg.RosPack().get_path('curve_generator') + "/src")
from CurveGen import CurveGen
import numpy as np

curve_name = "curve_03"
data_path = rospkg.RosPack().get_path('planning_utils')+'/data' + "/" + curve_name 
# path_file = "path_refine.mat"
path_file = "statistic_data.mat"

## get the path from the matlab file
path_refine = loadmat(data_path + "/" + path_file)


uv_path = path_refine['path_refine']

curve_file = data_path + "/" + curve_name + ".json"

cg = CurveGen(curve_file)
iks = IkSolver(cg)


# ///////////////// DEBUG ///////////////////////
# line1_v = np.arange(0.2,0.8,0.01)
# line1_u = np.full(len(line1_v),0.2)

# line2_v = np.arange(0.8,0.2,-0.01)
# line2_u = np.arange(0.2,0.8,0.01)

# line_u = np.concatenate((line1_u, line2_u))
# line_v = np.concatenate((line1_v, line2_v))
# uv_path = np.vstack((line1_v, 1 -line1_u))

# line1_u = np.linspace(uv_path[0][0], uv_path[0][-1], len(uv_path[0]))
# line1_v = np.linspace(uv_path[1][0], uv_path[1][-1], len(uv_path[1]))
# uv_path = np.vstack((line1_u, line1_v))
# ///////////////// DEBUG //////////

# from uv path to the se3 path 
se3_path = []
for i in range(len(uv_path[0])):
    se3_path.append(iks.uv_to_se3(uv_path[0][i], uv_path[1][i]))

# from se3 path to the state path 
state_path = []
for i in range(len(se3_path)):
    state_path.append(iks.se3_to_state(se3_path[i][0], se3_path[i][1]))

# save the se3 path and state path 
np.save(data_path + "/se3_path.npy", se3_path)
np.save(data_path + "/state_path.npy", state_path)

savemat(data_path + "/path.mat", {'uv_path': uv_path, 'se3_path': se3_path, 
                                  'state_path': state_path})


