import rospkg
import sys
sys.path.append(rospkg.RosPack().get_path("curve_generator") + "/src")
from CurveGen import CurveGen
from scipy.io import savemat
from invkin import IkSolver

data_path = rospkg.RosPack().get_path("planning_utils") + "/data/"
curve_name = "curve_03"
file_path = data_path + "/" + curve_name + "/" + curve_name + ".json"

cg = CurveGen(file_path)
iks = IkSolver(cg)


# //////////////// Debug ////////////////////////////////
xi = [0.25, 0.7]
xg = [0.75, 0.3]

pos, norm = iks.uv_to_se3(xi[0],xi[1])
strat_state = iks.se3_to_state(pos, norm)

pos, norm = iks.uv_to_se3(xg[0],xg[1])
goal_state = iks.se3_to_state(pos, norm)


# get the data, such as curve points, normals, curvatures, etc.
rsl = 0.01
u_range = (0.2,0.8)
v_range = (0.2,0.8)

u_range = range(int(u_range[0]/rsl), int(u_range[1]/rsl))
v_range = range(int(v_range[0]/rsl), int(v_range[1]/rsl))

points, normals, curvatures, state = [], [], [], []
points_temp, normals_temp, curvatures_temp, state_temp = [], [], [], []

for i in u_range:
    for j in v_range:
        points_temp.append(cg.getPointPos(i*rsl, j*rsl))
        normals_temp.append(cg.getPointNormal(i*rsl, j*rsl))
        curvatures_temp.append(cg.getPointCurvature(i*rsl, j*rsl))
        pos, norm = iks.uv_to_se3(i*rsl, j*rsl)
        state_temp.append(iks.se3_to_state(pos, norm))
    points.append(points_temp)
    normals.append(normals_temp)
    curvatures.append(curvatures_temp)
    state.append(state_temp)
    points_temp, normals_temp, curvatures_temp, state_temp = [], [], [], []

# save the data as a mat file
savemat(data_path + curve_name + "/" + curve_name + "_data.mat", \
        {"points": points, "normals": normals, "curvatures": curvatures,\
        "state_map": state, "start_state": strat_state, "goal_state": goal_state})