import rospy
import os
import numpy as np
from uam_control import UamControl
from std_msgs.msg import Bool

base_path = os.path.dirname(__file__) + '/../../../'

def msg_out(msg):
    global last_req
    if (rospy.Time.now() - last_req).to_sec() > 2:
        last_req = rospy.Time.now()
        rospy.loginfo(msg)

# import the traj data
traj_data = np.load(base_path + 'data/state_traj_pos.npy')

# start and goal pose
start_pose = traj_data[0, :]
start_yaw = 0.0
start_thresh = 0.5

target_pose = traj_data[-1, :]
target_yaw = 0.0

# devide the traj into separate direction
traj_x = traj_data[:, 0]
traj_y = traj_data[:, 1]
traj_z = traj_data[:, 2]

# set up the publishers and parameters
task_pub = rospy.Publisher('uam_control_interface/in_task', Bool, queue_size=1)
rospy.set_param('recorded_traj', False)

# other parameters
count = 0

if __name__ == '__main__':

    uc = UamControl()

    offboard_state = False
    reach_start = False
    reach_target = False

    rate = rospy.Rate(10)

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
            uc.set_joint_pos(-np.pi/2)
            msg_out("moving to start pose")
            if (np.linalg.norm(np.array(uc.get_current_config()[0] - start_pose)) < start_thresh):
                if not reach_start:
                    rospy.set_param('record_traj', True)
                reach_start = True

        if reach_start and (not reach_target):
            uc.set_pose(traj_x[count], traj_y[count], traj_z[count], 0)
            uc.set_joint_pos(-np.pi/2)
            rospy.loginfo("set state pos: ({}, {}, {})".format( \
                traj_x[count], traj_y[count], traj_z[count]))
            count += 1
            msg_out("moving along the trajectory")
            if (count >= traj_x.shape[0]):
                if not reach_target:
                    rospy.set_param('record_traj', False)
                reach_target = True

        if reach_target:
            uc.set_pose(target_pose[0], target_pose[1], target_pose[2], target_yaw)
            uc.set_joint_pos(-np.pi/2)
            msg_out("moving to target pose")

        rate.sleep()


