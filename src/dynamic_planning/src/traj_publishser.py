#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class TrajectoryPublisher:
    def __init__(self):
        self.ref_pos_sub = rospy.Subscriber('/reference_pos', Float32, self.ref_pos_callback)
        self.target_pos_pub = rospy.Publisher('/target_trajectory', Float32MultiArray, queue_size=10)
        self.target_pos_offset = 0

        ## Load the path csv file

    def ref_pos_callback(self, msg):
        self.target_pos_offset = msg.data

        
    def publish_target_trajectory(self):
        target_pos_msg = Float32MultiArray()
        target_pos_msg.data = [0.6+self.target_pos_offset, 0, 1.5, 0, 0, 0]
        self.target_pos_pub.publish(target_pos_msg)
        print(f"publishing target_pos_msg: {target_pos_msg.data}")


if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    traj_publisher = TrajectoryPublisher()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        traj_publisher.publish_target_trajectory()


        

# Calculates the distance between two points

# Subdivides accourding to the distance
