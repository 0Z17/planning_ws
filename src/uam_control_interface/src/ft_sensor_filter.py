#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, filtfilt
import numpy as np

# 设置低通滤波器的参数
FILTER_CUTOFF = 10.0  # 截止频率 (Hz)
FILTER_ORDER = 4     # 滤波器阶数
SAMPLING_RATE = 50   # 采样频率 (Hz)，根据实际传感器数据调整

# 低通滤波器设计
def butter_lowpass(cutoff, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=4):
    b, a = butter_lowpass(cutoff, fs, order)
    return filtfilt(b, a, data)

# 用于存储z轴的力数据
force_z_data = []

# 创建一个Publisher对象
pub = None

# 回调函数，获取/ft_sensor_topic的消息
def ft_sensor_callback(msg):
    global force_z_data, pub
    
    # 获取z轴的力
    force_z = msg.wrench.force.z
    
    # 将z轴力数据添加到列表
    force_z_data.append(force_z)
    
    # 如果列表中有足够的样本数据，则进行滤波
    if len(force_z_data) > SAMPLING_RATE:  # 只有采样足够时进行滤波
        # 执行低通滤波
        filtered_data = butter_lowpass_filter(np.array(force_z_data), FILTER_CUTOFF, SAMPLING_RATE, FILTER_ORDER)
        # 输出滤波后的最后一个数据点
        rospy.loginfo(f"Filtered Force (z): {filtered_data[-1]}")
        
        # 发布滤波后的消息
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        filtered_msg.wrench.force.z = filtered_data[-1]
        pub.publish(filtered_msg)
        
        # 清空数据列表以便处理下一个周期的数据
        force_z_data.clear()  # 使用clear()来清空列表

def main():
    global pub
    # 初始化ROS节点
    rospy.init_node('ft_sensor_lowpass_filter', anonymous=True)
    
    # 创建Publisher对象，用于发布滤波后的数据
    pub = rospy.Publisher('/filtered_ft_sensor_topic', WrenchStamped, queue_size=10)
    
    # 订阅 /ft_sensor_topic 话题
    rospy.Subscriber('/ft_sensor_topic', WrenchStamped, ft_sensor_callback)
    
    # 持续运行ROS节点
    rospy.spin()

if __name__ == '__main__':
    main()
