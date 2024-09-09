import numpy as np
import rospkg
import sys

sys.path.append(rospkg.RosPack().get_path("curve_generator") + "/src")
from CurveGen import CurveGen
from invkin import IkSolver
from scipy.io import savemat

# load the curve data 
curve_name = "curve_simple"
data_path = rospkg.RosPack().get_path("planning_utils") + "/data/" + curve_name
file_name = data_path + "/"  + curve_name + ".json"

cg = CurveGen(file_name)
iks = IkSolver(cg)

# get the partial derivative of the curve
rsl = 0.01
u_range = (0.2,0.8)
v_range = (0.2,0.8)

u_range = range(int(u_range[0]/rsl), int(u_range[1]/rsl))
v_range = range(int(v_range[0]/rsl), int(v_range[1]/rsl))

map_du, map_dv, map_du_c, map_dv_c  = [], [], [], []

for i in u_range:
    for j in v_range:
        map_du_c.append(iks.zeta_to_vse3(i*rsl, j*rsl, 1, 0))
        map_dv_c.append(iks.zeta_to_vse3(i*rsl, j*rsl, 0, 1))
    map_du.append(map_du_c)
    map_dv.append(map_dv_c)
    map_du_c = []
    map_dv_c = []

# save the map as a numpy array
map_du = np.array(map_du)
map_dv = np.array(map_dv)
np.save(data_path + "/" + curve_name + "_map.npy", np.stack([map_du, map_dv], axis=2))

# save the map as a mat file
savemat(data_path + "/" + curve_name + "_map.mat", {"curveMap": np.stack([map_du, map_dv], axis=2)})