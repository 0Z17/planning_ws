from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import rospkg
import time
from matplotlib import pyplot as plt
from scipy.io import savemat

data_path = rospkg.RosPack().get_path('planning_utils') + '/data/curve_large/'


class Bar:
    def __init__(self, name='/Bar'):

        client = RemoteAPIClient("172.30.144.1")
        self.sim = client.require('sim')

        self.handle = self.sim.getObject(name)
        self.x_joint = self.sim.getObject('/x_joint')
        self.y_joint = self.sim.getObject('/y_joint')
        self.z_joint = self.sim.getObject('/z_joint')
        self.yaw_joint = self.sim.getObject('/yaw_joint')
        self.theta_joint = self.sim.getObject('/theta_joint')

    def set_pose(self, x, y, z, yaw, pitch):
        self.sim.setJointPosition(self.x_joint, x)
        self.sim.setJointPosition(self.y_joint, y)
        self.sim.setJointPosition(self.z_joint, z)
        self.sim.setJointPosition(self.yaw_joint, yaw)
        self.sim.setJointPosition(self.theta_joint, -pitch)

    def set_velocity(self, vx, vy, vz, vyaw, vpitch):
        self.sim.setJointTargetVelocity(self.x_joint, vx)
        self.sim.setJointTargetVelocity(self.y_joint, vy)
        self.sim.setJointTargetVelocity(self.z_joint, vz)
        self.sim.setJointTargetVelocity(self.yaw_joint, vyaw)
        self.sim.setJointTargetVelocity(self.theta_joint, -vpitch)

    def get_velocity(self):
        vx = self.sim.getJointVelocity(self.x_joint)
        vy = self.sim.getJointVelocity(self.y_joint)
        vz = self.sim.getJointVelocity(self.z_joint)
        vyaw = self.sim.getJointVelocity(self.yaw_joint)
        vpitch = -self.sim.getJointVelocity(self.theta_joint)
        return np.array([vx, vy, vz, vyaw, vpitch])

if __name__ == '__main__':
    bar = Bar()
    se3_traj = np.load(data_path + 'se3_traj.npy')
    state_traj = np.load(data_path + 'state_traj.npy')
    end_vel_traj = np.load(data_path + 'se3_vel_traj.npy')


    pos_ls = se3_traj[:,0]
    rot_ls = state_traj[:,3:5]

    bar.set_pose(se3_traj[0][0][0], se3_traj[0][0][1], se3_traj[0][0][2], state_traj[0][3], state_traj[0][4])
    bar.set_velocity(0,0,0,0,0)

    # plot the velocity in separate subplots

    # plt.figure()
    # plt.subplot(5,1,1)
    # plt.plot(end_vel_traj[:,0])
    # plt.subplot(5,1,2)
    # plt.plot(end_vel_traj[:,1])
    # plt.subplot(5,1,3)
    # plt.plot(end_vel_traj[:,2])
    # plt.subplot(5,1,4)
    # plt.plot(end_vel_traj[:,3])
    # plt.subplot(5,1,5)
    # plt.plot(end_vel_traj[:,4])
    yaw = rot_ls[:,0]

    for i in range(len(yaw)):
        if yaw[i] > 3/2*np.pi:
            yaw[i] = - (2*np.pi - yaw[i])


    plt.subplot(5,1,1)
    diff_x = np.diff(pos_ls[:,0])*10
    diff_x = np.diff(diff_x)*10
    up_bound = np.full(len(diff_x), 0.4)
    plt.plot(up_bound, 'r--')
    plt.plot(diff_x)
    plt.subplot(5,1,2)
    diff_y = np.diff(pos_ls[:,1])*10
    diff_y = np.diff(diff_y)*10
    up_bound = np.full(len(diff_x), 0.4)
    plt.plot(up_bound, 'r--')
    plt.plot(diff_y)
    plt.subplot(5,1,3)
    diff_z = np.diff(pos_ls[:,2])*10
    diff_z = np.diff(diff_z)*10
    up_bound = np.full(len(diff_x), 0.4)
    plt.plot(up_bound, 'r--')
    plt.plot(diff_z)
    plt.subplot(5,1,4)
    diff_yaw = np.degrees(np.diff(yaw)*10)
    diff_yaw = np.diff(diff_yaw)*10
    yaw_up_bound = np.full(len(diff_x), 10)
    yaw_low_bound = np.full(len(diff_x), -10)
    plt.plot(yaw_up_bound, 'r--')
    plt.plot(yaw_low_bound, 'r--')
    plt.plot(diff_yaw)
    plt.subplot(5,1,5)
    diff_pitch = np.degrees(np.diff(rot_ls[:,1])*10)
    diff_pitch = np.diff(diff_pitch)*10
    up_bound = np.full(len(diff_x), 10)
    low_bound = np.full(len(diff_x), -10)
    plt.plot(up_bound, 'r--')
    plt.plot(low_bound, 'r--')
    plt.plot(diff_pitch)

    savemat(data_path + 'diff_traj.mat', {'diff_x': diff_x, 'diff_y': diff_y, 'diff_z': diff_z, 'diff_yaw': diff_yaw, 'diff_pitch': diff_pitch})

    plt.show()

    real_vel = []

    bar.set_pose(pos_ls[0][0], pos_ls[0][1], pos_ls[0][2], rot_ls[0][0], rot_ls[0][1])


    
    for i in range(len(pos_ls)):
        bar.set_pose(pos_ls[i][0], pos_ls[i][1], pos_ls[i][2], rot_ls[i][0], rot_ls[i][1])
        time.sleep(0.05)


    bar.sim.setStepping(True)
    bar.sim.startSimulation()
    # sim_time = bar.sim.getSimulationTime()
    # i = 0

    # while bar.sim.getSimulationTime() < len(pos_ls)*0.1:
    #     if bar.sim.getSimulationTime() - sim_time > 0.1:
    #         sim_time = bar.sim.getSimulationTime()
    #         print('Sim time: ', sim_time)
    #         real_vel.append(bar.get_velocity())
    #         i += 1
    #     # print('Sim time: ', sim_time)
    #     bar.set_velocity(end_vel_traj[i][0], end_vel_traj[i][1], end_vel_traj[i][2], end_vel_traj[i][3], end_vel_traj[i][4])
    #     bar.sim.step()

    # print(i)
    # bar.sim.stopSimulation()
        

    # real_vel = np.array(real_vel)
    # plt.subplot(5,1,1)
    # plt.plot(real_vel[:,0],'r')
    # plt.subplot(5,1,2)
    # plt.plot(real_vel[:,1],'r')
    # plt.subplot(5,1,3)
    # plt.plot(real_vel[:,2],'r')
    # plt.subplot(5,1,4)
    # plt.plot(real_vel[:,3],'r')
    # plt.subplot(5,1,5)
    # plt.plot(real_vel[:,4],'r')

    # bar.sim.stopSimulation()

    plt.show()

    # ///////////////////////////////////////////////////////////////////



