#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf_conversions

# static tf params
trans_ls = {}
rot_ls = {}

link_name = [f"rotor_{_}_collision_respondable" for _ in range(6)]

# robot_base to rotor_0_collision_respondable
trans_ls[link_name[0]] =    [-2.443805,  -1.735699,   -1.327300]
trans_ls[link_name[1]] =    [-0.975004,  -2.521752,   -1.727840]
trans_ls[link_name[2]] =    [-1.308067,  -2.384135,   0.404736]
trans_ls[link_name[3]] =    [-1.540114,  -1.213949,   -1.327299]
trans_ls[link_name[4]] =    [-0.975004,  -1.478251,   -1.727840]
trans_ls[link_name[5]] =    [-2.211756,  -1.862385,   0.404736]

rot_ls[link_name[0]] =      [-2.677950,  2.888912 ,   -3.081642]
rot_ls[link_name[1]] =      [3.141592 ,  3.665189 ,   -3.141592]
rot_ls[link_name[2]] =      [2.677949 ,  2.888913 ,   3.081642]
rot_ls[link_name[3]] =      [-2.677950,  2.888912 ,   -3.081642]
rot_ls[link_name[4]] =      [3.141592 ,  3.665189 ,   -3.141592]
rot_ls[link_name[5]] =      [2.677949 ,  2.888913 ,   3.081642]


base_link_pose = PoseStamped()

def local_pos_cb(msg):
    global base_link_pose
    base_link_pose = msg

def gen_transform_rpy(x, y, z, roll, pitch, yaw, child_frame_id, parent_frame_id):

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame_id
    t.child_frame_id = child_frame_id

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    quat = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    return t

def gen_transform_quad(x,y,z,ori, child_frame_id, parent_frame_id):

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame_id
    t.child_frame_id = child_frame_id


    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z

    t.transform.rotation = ori

    return t

def broadcast_transform():
    rospy.init_node('map_to_robot_base_tf_broadcaster')

    # create a tf2 broadcaster
    br = tf2_ros.TransformBroadcaster()

    # set the rate 
    rate = rospy.Rate(50)  # 10 Hz

    # initialize the subscriber
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_pos_cb)

    while not rospy.is_shutdown():

        # static tf
        
        for i in range(6):
            t = gen_transform_rpy(trans_ls[link_name[i]][0], trans_ls[link_name[i]][1], trans_ls[link_name[i]][2], 
                              rot_ls[link_name[i]][0], rot_ls[link_name[i]][1], rot_ls[link_name[i]][2], link_name[i], "robot_base")
            br.sendTransform(t)

        # [map] to [base_link] tf
        t = gen_transform_quad(base_link_pose.pose.position.x, base_link_pose.pose.position.y, base_link_pose.pose.position.z, 
                              base_link_pose.pose.orientation, "robot_base", "map")
        br.sendTransform(t)

        # [base_link] to [operator_1_Link_collision] tf
        t = gen_transform_rpy(trans_ls[link_name[1]][0], trans_ls[link_name[1]][1], trans_ls[link_name[1]][2], 
                              rot_ls[link_name[1]][0], rot_ls[link_name[1]][1], rot_ls[link_name[1]][2], "operator_1_Link_collision", "robot_base")
        br.sendTransform(t)

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
