#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32
import numpy as np

class ImpedenceControl:
    def __init__(self):
        self.sub = rospy.Subscriber("/filtered_ft_sensor_topic", WrenchStamped, self.wrench_callback)
        self.pos_pub = rospy.Publisher("/reference_pos", Float32, queue_size=10)

        # The measured force
        self.fe = 0.0
        # self.f_offset = -1.5
        self.f_offset = -2.5

        # Reference pos and vel in the previous time step
        self.pr_last = 0.0
        self.vr_last = 0.0

        # The desired pos, vel, and acc
        self.pd = 0.0
        self.vd = 0.0
        self.ad = 0.0

        # The peremeter of the impedence dynamic model
        self.M = 1.0
        self.D = 50.0
        # self.K = 30.0
        self.K = 5.0   # //// a feasible parameter ////
        # self.K = 10.0

       # The time step
        self.dt = 0.01

    def wrench_callback(self, msg):
        """
        The callback function for the force topic
        """
        self.fe = msg.wrench.force.z + self.f_offset


    def update(self):
        """
        The impedence control algorithm
        """
        pr = (self.dt * (- self.fe * self.dt + self.K * self.pd * self.dt + self.D * (self.pr_last + self.dt * self.vd)) + self.M * (self.pr_last + self.dt * (self.ad * self.dt + self.vr_last))) / (self.M + self.dt * (self.D + self.K * self.dt))

        self.vr_last = (pr - self.pr_last) / self.dt
        self.pr_last = pr

        

if __name__ == '__main__':
    rospy.init_node('impedence_control')
    ic = ImpedenceControl()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        ic.update()
        ic.pos_pub.publish(ic.pr_last)
        print("pr: ", ic.pr_last)
        rate.sleep()