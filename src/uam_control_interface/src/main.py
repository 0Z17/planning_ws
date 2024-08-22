#!/usr/bin/env python3

import rospy
from uam_control import UamControl
from std_msgs.msg import Bool, Float32
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import os, sys
import numpy as np
base_path = os.path.dirname(__file__) + '/../../../'
sys.path.append(base_path + "src/control_aware_planner/src/")
from invkin import IkSolver

def msg_out(msg):
    global last_req
    if (rospy.Time.now() - last_req).to_sec() > 2:
        last_req = rospy.Time.now()
        rospy.loginfo(msg)

if __name__ == '__main__':


    uc = UamControl()

    task_pub = rospy.Publisher('uam_control_interface/in_task', Bool, queue_size=1)
    # traj = np.load(base_path + 'data/state_traj.npy')

    count = 0

    frq = 200
    rate = rospy.Rate(frq)
    dt = 1.0/frq

    # get the disered pose and volecity
    start_pose = np.array([0, 0, 2])
    start_yaw = 0
    start_joint_pos = 0
    target_pose = np.array([0, 1, 2])
    target_yaw = 0
    target_joint_pos = 3/2 * np.pi

    start_thresh = 0.5


    # set offboard mode
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
 
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not uc.current_state.connected):
        rate.sleep()

    # processing time
    t_proc = 30.0

    # interplation the trajectory
    iks = IkSolver()


    val_x = [start_pose[0], target_pose[0]]
    val_y = [start_pose[1], target_pose[1]]
    val_z = [start_pose[2], target_pose[2]]
    val_yaw = [start_yaw, target_yaw]
    val_joint_pos = [start_joint_pos, target_joint_pos]
    
    traj_x, _ = iks.time_alloc(val_x, int(t_proc * frq), 'linear')
    traj_y, _ = iks.time_alloc(val_y, int(t_proc * frq), 'linear')
    traj_z, _ = iks.time_alloc(val_z, int(t_proc * frq), 'linear')
    traj_yaw, _ = iks.time_alloc(val_yaw, int(t_proc * frq), 'linear')
    traj_joint, _ = iks.time_alloc(val_joint_pos, int(t_proc * frq), 'linear')
    
    offboard_state = False
    reach_start = False
    reach_target = False

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():
        task_pub.publish(Bool(True))

        if(uc.current_state.mode!= "OFFBOARD"):
            uc.set_pose(start_pose[0], start_pose[1], start_pose[2], start_yaw)
            last_req = rospy.Time.now()
            msg_out("waiting for OFFBOARD mode")
        elif(uc.current_state.mode == "OFFBOARD"):
            offboard_state = True

        # check if the current pose arrived the start pose
        if offboard_state and (not reach_start):
            uc.set_pose(start_pose[0], start_pose[1], start_pose[2], start_yaw)
            # uc.set_joint_pos(start_joint_pos)
            msg_out("moving to start pose")
            if (np.linalg.norm(np.array(uc.get_current_config()[0] - start_pose)) < start_thresh):
                reach_start = True


        if reach_start and (not reach_target):
            uc.set_pose(traj_x[count], traj_y[count], traj_z[count], traj_yaw[count])
            # uc.set_joint_pos(traj_joint[count])
            rospy.loginfo("set state pos: ({}, {}, {}, {}, {})".format( \
                traj_x[count], traj_y[count], traj_z[count], traj_yaw[count], traj_joint[count]))
            count += 1
            msg_out("moving along the trajectory")
            if (count >= traj_x.shape[0]):
                reach_target = True

        if reach_target:
            uc.set_pose(target_pose[0], target_pose[1], target_pose[2], target_yaw)
            # uc.set_joint_pos(target_joint_pos)
            msg_out("moving to target pose")

        rate.sleep()
