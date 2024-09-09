# test the normal deriv
import numpy as np
import rospkg
import sys
import time
from scipy.io import savemat


sys.path.append(rospkg.RosPack().get_path('curve_generator') + '/src')
sys.path.append(rospkg.RosPack().get_path('control_aware_planner') + '/src')

from CurveGen import CurveGen
from invkin import IkSolver

# create a CurveGen object
data_path = rospkg.RosPack().get_path('planning_utils') + '/data/curve_large'
curve_file = data_path + '/curve_large.json'

cg = CurveGen(file_name=curve_file)
iks = IkSolver(cg)

cur_ls = []
norm_ls = []
pos_ls = []
dpos_du_ls = []
dpos_dv_ls = []
dnorm_du_ls = []
dnorm_dv_ls = []
dpsi_du_ls = []
dpsi_dv_ls = []
dtheta_du_ls = []
dtheta_dv_ls = []
psi_ls = [] 
theta_ls = []
dpsi_du_ls = []
dpsi_dv_ls = []
dtheta_du_ls = []
dtheta_dv_ls = []

resol = 0.01

# record the runnig time
start_time = time.time()
for i in range(20,80):
    for j in range(20,80):
        cur_ls.append(cg.getPointCurvature(i*resol, j*resol))
        norm = cg.getPointNormal(i*resol, j*resol)
        norm_ls.append(norm)
        pos = cg.getPointPos(i*resol, j*resol)
        pos_ls.append(pos)
        deriv = cg.getPointDeriv(i*resol, j*resol)
        dpos_du_ls.append(deriv[1][0])
        dpos_dv_ls.append(deriv[0][1])
        dnorm = cg.getPointNormalDeriv(i*resol, j*resol)
        dnorm_du_ls.append(dnorm[0])
        dnorm_dv_ls.append(dnorm[1])
        state = iks.se3_to_state(pos, norm)
        psi_ls.append(state[3])
        theta_ls.append(state[4])
        dpsi_du_ls.append(iks.get_dpsi(norm,dnorm,1,0))
        dpsi_dv_ls.append(iks.get_dpsi(norm,dnorm,0,1))
        dtheta_du_ls.append(iks.get_dtheta(norm,dnorm,1,0))
        dtheta_dv_ls.append(iks.get_dtheta(norm,dnorm,0,1))

time_elapsed = time.time() - start_time
print('Time elapsed: %.2f s' % time_elapsed)

savemat(data_path + '/uv_test.mat', {'cur_ls': cur_ls, 'norm_ls': norm_ls,
                                    'pos_ls': pos_ls, 'dpos_du_ls': dpos_du_ls,
                                    'dpos_dv_ls': dpos_dv_ls, 'dnorm_du_ls': dnorm_du_ls, 
                                    'dnorm_dv_ls': dnorm_dv_ls, 'dpsi_du_ls': dpsi_du_ls,
                                    'dpsi_dv_ls': dpsi_dv_ls, 'dtheta_du_ls': dtheta_du_ls,
                                    'dtheta_dv_ls': dtheta_dv_ls, 'psi_ls': psi_ls, 
                                    'theta_ls': theta_ls})

