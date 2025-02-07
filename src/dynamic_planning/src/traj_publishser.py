#!/usr/bin/env python3

import rospy
import pandas as pd
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import numpy as np

class TrajectoryPublisher:
    def __init__(self):
        self.ref_pos_sub = rospy.Subscriber('/reference_pos', Float32, self.ref_pos_callback)
        self.target_pos_pub = rospy.Publisher('/target_trajectory', Float32MultiArray, queue_size=10)
        self.joint_pos_sub = rospy.Publisher('/skyvortex/operator_1_joint/pos_cmd', Float32, queue_size=10)
        self.refine_path = []
        self.target_config_offset = 0
        self.weight = np.array([1, 1, 1, 1, 3])
        self.cost_res = 0.001
        # self.cost_res = 0.0005
        self.pub_count = 0
        self.joint_pos_offset = -np.pi/6

        ## Load the path csv file

    def ref_pos_callback(self, msg):
        self.target_config_offset = msg.data

        
    def publish_target_trajectory(self):
        target_pos_msg = Float32MultiArray()
        if self.pub_count >= len(self.refine_path):
            return
        normal_vec = self.normal_vec(self.refine_path[self.pub_count])
        target_pos_msg.data = np.array(self.refine_path[self.pub_count]) + \
                                (self.target_config_offset * np.array(normal_vec + [0, 0]))
        target_pos_msg.data[4] += self.joint_pos_offset
        self.pub_count += 1
        self.target_pos_pub.publish(target_pos_msg)
        joint_pos_msg = Float32()
        joint_pos_msg.data =  target_pos_msg.data[4]
        self.joint_pos_sub.publish(joint_pos_msg)
        print(f"publishing target_pos_msg: {target_pos_msg.data}")

    def publish_target_start(self):
        target_pos_msg = Float32MultiArray()
        normal_vec = self.normal_vec(self.refine_path[0])
        target_pos_msg.data = np.array(self.refine_path[0]) + \
                                (self.target_config_offset * np.array(normal_vec + [0, 0]))
        target_pos_msg.data[4] += self.joint_pos_offset
        self.target_pos_pub.publish(target_pos_msg)
        joint_pos_msg = Float32()
        joint_pos_msg.data =  target_pos_msg.data[4]
        self.joint_pos_sub.publish(joint_pos_msg)
        print(f"publishing start_pos_msg: {target_pos_msg.data}")
        
    def load_path_csv(self, path_csv_file):
        self.path_df = pd.read_csv(path_csv_file, header=0)
        self.path = np.array(self.path_df.values.tolist())

    def normal_vec(self, config): 
        psi = config[3]
        theta = config[4]
        normal_vec = [np.cos(psi)*np.cos(theta), np.sin(psi)*np.cos(theta), np.sin(theta)]
        return normal_vec

    def path_interpolation(self):
        for i in range(len(self.path)-1):
            cost = self.calculate_cost([self.path[i], self.path[i+1]])  # calculate the cost of the segment
            interpolate = np.linspace(self.path[i], self.path[i+1], num=int(np.ceil(cost/self.cost_res)))  # subdivide the segment
            self.refine_path.extend(interpolate[:-1].tolist())  # add the subdivided segment to the refine_path list
        self.refine_path.append(self.path[-1])  # add the last point of the path to the refine_path list

    def calculate_cost(self, segment):
        return np.linalg.norm(self.weight * (segment[1] - segment[0]))  # assing weights to each dimension and calculate the cost


if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    tp = TrajectoryPublisher()
    # tp.load_path_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_PCSFMT.csv")
    tp.load_path_csv("/home/wsl/proj/my_ompl/demos/MyPlanners/test_output/state_path_FMT.csv")
    tp.path_interpolation()
    print(f"refine_path: {tp.refine_path}")
    rate = rospy.Rate(50)

    # publish the start position for 10 seconds
    start_time = rospy.Time.now()
    while ((rospy.Time.now() - start_time).to_sec() < 10 ) and (not rospy.is_shutdown()) :
        tp.publish_target_start()
        rate.sleep()

    while not rospy.is_shutdown():
        tp.publish_target_trajectory()
        rate.sleep()


        

# Calculates the distance between two points

# Subdivides accourding to the distance
