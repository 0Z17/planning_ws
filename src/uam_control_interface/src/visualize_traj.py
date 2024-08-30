#!/usr/bin/env python3

import rospy, rospkg
import os
import numpy as np
from std_msgs.msg import Bool
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

rospkg = rospkg.RosPack()
utils_path = rospkg.get_path('planning_utils')

rospy.init_node('visualize_end')

# variables initialization
end_pose_list = []
body_pose_list = []
end_pose = None
body_pose = None

# gazebo params about the operation link
body_link_id = 3
end_link_id = 5

# other params
last_request = rospy.Time.now()

def task_cb(msg):
    global last_request
    is_intask = msg.data
    if is_intask and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
        rospy.loginfo("In task")
        last_request = rospy.Time.now()
    
    is_record = rospy.get_param('record_traj', False)
    if is_record and (end_pose is not None):
        end_pose_list.append(end_pose)
        rospy.loginfo("End pose added to list: {}".format(end_pose))
    if is_record and (body_pose is not None):
        body_pose_list.append(body_pose)
        rospy.loginfo("Body pose added to list: {}".format(body_pose))
        
def end_cb(msg):
    global end_pose
    end_pose_raw = msg.pose[end_link_id]
    end_pose = [end_pose_raw.position.x, 
                end_pose_raw.position.y,
                end_pose_raw.position.z]
    
def body_cb(msg):
    global body_pose
    body_pose_raw = msg.pose[body_link_id]
    body_pose = [body_pose_raw.position.x, 
                 body_pose_raw.position.y,
                 body_pose_raw.position.z]
    
def gen_pose(x , y, z):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    return pose_msg
    
# import the traj data
traj_data = np.load(utils_path + '/data/state_traj.npy')

# setup subscribers
tasksub = rospy.Subscriber('/uam_control_interface/in_task', Bool, task_cb)
endpos_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, end_cb)
bodypos_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, body_cb)

# setup publisher
desired_end_path_pub = rospy.Publisher('/uam_control_interface/desired_end_path', Path, queue_size=10)
desired_body_path_pub = rospy.Publisher('/uam_control_interface/desired_body_path', Path, queue_size=10)
real_end_path_pub = rospy.Publisher('/uam_control_interface/real_end_path', Path, queue_size=10)
real_body_path_pub = rospy.Publisher('/uam_control_interface/real_body_path', Path, queue_size=10)

rate = rospy.Rate(10)

# initialize the path msg
desired_end_path_msg = Path()
desired_end_path_msg.header.stamp = rospy.Time.now()
desired_end_path_msg.header.frame_id = "map"

desired_body_path_msg = Path()
desired_body_path_msg.header.stamp = rospy.Time.now()
desired_body_path_msg.header.frame_id = "map"

real_end_path_msg = Path()
real_end_path_msg.header.stamp = rospy.Time.now()
real_end_path_msg.header.frame_id = "map"

real_body_path_msg = Path()
real_body_path_msg.header.stamp = rospy.Time.now()
real_body_path_msg.header.frame_id = "map"

# loop until the end of the trajectory
for i in range(traj_data.shape[0]):

    # # set the desired end pose
    # desired_end_pose_msg = gen_pose(traj_data[count, 0], traj_data[count, 1], traj_data[count, 2])
    # desired_end_path_msg.poses.append(desired_end_pose_msg)

    # set the desired body pose
    desired_body_pose_msg = gen_pose(traj_data[i, 0], traj_data[i, 1], traj_data[i, 2])
    desired_body_path_msg.poses.append(desired_body_pose_msg)

while not rospy.is_shutdown():

    # publish the desired body path
    desired_body_path_pub.publish(desired_body_path_msg)

    is_record = rospy.get_param('record_traj', False)

    # publish the real body path
    if is_record and (body_pose is not None):
        real_body_pose_msg = gen_pose(body_pose[0], body_pose[1], body_pose[2])
        real_body_path_msg.poses.append(real_body_pose_msg)
    real_body_path_pub.publish(real_body_path_msg)

    if is_record and (end_pose is not None):
        # publish the desired end path
        real_end_pose_msg = gen_pose(end_pose[0], end_pose[1], end_pose[2])
        real_end_path_msg.poses.append(real_end_pose_msg)
    real_end_path_pub.publish(real_end_path_msg)

    rate.sleep()