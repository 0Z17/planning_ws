#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from uam_control_interface.msg import SetPosition
import numpy as np


class JointMsgConvert:
    def __init__(self):
        rospy.init_node('joint_msg_convert')


        self.JOINT_POS_INIT = 6144
        self.JOINT_POS_INC = -5120/(np.pi/2)
        self.joint_pos = None
        self.joint_offset = -np.pi/6

        self.joint_name = "operator_1_joint"
        self.vehicle_name = "skyvortex"

        self.joint_num_sub = rospy.Subscriber("/set_position", SetPosition, self.joint_num_cb)
        self.joint_pos_pub = rospy.Publisher(self.vehicle_name + "/" + self.joint_name + "/pos_cmd", Float32, queue_size=10)

    def joint_num_cb(self, msg):
        self.joint_pos = (msg.position - self.JOINT_POS_INIT ) / self.JOINT_POS_INC 

    def pub_joint_pos(self):
        if self.joint_pos is not None:
            self.joint_pos_pub.publish(self.joint_pos + self.joint_offset)
            
            

if __name__ == '__main__':
    jmc = JointMsgConvert()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        jmc.pub_joint_pos()
        rate.sleep()
    