import sys
import rospkg
import rospy
import numpy as np
from geomdl import exchange
from scipy.io import savemat

rospkg = rospkg.RosPack()
package_path = rospkg.get_path('curve_generator')
sys.path.append(package_path + '/src')

from CurveGen import CurveGen
from invkin import IkSolver
from visCurve import vis_curve



if __name__ == '__main__':
    curve_name = 'curve_simple'
    data_path = rospkg.get_path('planning_utils') + '/data' + '/' + curve_name
    file_name = data_path +'/' + curve_name + '.json'

    cg = CurveGen(file_name)
    iks = IkSolver()

    # cg.visualizeCurve()

    # cg.curve = exchange.import_json(data_path + '/curve_large/curve_large.json')    
    iks.update_curve(cg)
    iks.set_link_lengths(0.923)

    # load the uv trajectory
    uv_traj = np.load(data_path + '/uv_traj_cur.npy')

    # convert the uv trajectory to se3 trajectory
    # se3_traj = []
    # state_traj = []
    # for i in range(len(uv_traj)):
    #     se3_waypt = iks.uv_to_se3(uv_traj[i][0][0], uv_traj[i][0][1])
    #     state_waypt = iks.se3_to_state(se3_waypt[0],se3_waypt[1])
    #     se3_traj.append(iks.uv_to_se3(uv_traj[i][0][0], uv_traj[i][0][1]))
    #     state_traj.append(state_waypt)
    # np.save(data_path + '/se3_traj.npy', se3_traj)
    # np.save(data_path + '/state_traj.npy', state_traj)

    # convert the se3 trajectory to state trajectory

    se3_traj = []
    se3_vel_traj = []
    state_traj = []
    for i in range(uv_traj.shape[1]):
        # se3_waypt = iks.uv_to_se3(uv_traj[i][0][0], uv_traj[i][0][1])
        se3_waypt = iks.uv_to_se3(uv_traj[0][i], uv_traj[1][i])
        state_waypt = iks.se3_to_state(se3_waypt[0],se3_waypt[1])
        # se3_vel = iks.zeta_to_vse3(uv_traj[i][0][0], uv_traj[i][0][1], uv_traj[i][1][0], uv_traj[i][1][1])
        se3_traj.append(se3_waypt)
        state_traj.append(state_waypt)
        # se3_vel_traj.append(se3_vel)
    
    np.save(data_path + '/se3_traj.npy', se3_traj)
    np.save(data_path + '/state_traj.npy', state_traj)
    # np.save(data_path + '/se3_vel_traj.npy', se3_vel_traj)
    savemat(data_path + '/traj_data.mat', {'se3_traj': se3_traj, 'state_traj': state_traj})
    # savemat(data_path + '/traj_data.mat', {'se3_traj': se3_traj, 'state_traj': state_traj, 'se3_vel_traj': se3_vel_traj})


                