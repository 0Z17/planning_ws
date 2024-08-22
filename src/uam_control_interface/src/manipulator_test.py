import rospy
from uam_control import UamControl
from std_msgs.msg import Bool, Float32
import os, sys
import numpy as np

pub = rospy.Publisher('/skyvortex/operator_1_joint/pos_cmd', Float32, queue_size=10)

while not rospy.is_shutdown():
    rospy.init_node('manipulator_test')

    pub_msg = Float32()
    pub_msg.data = 0.0
    pub.publish(pub_msg)