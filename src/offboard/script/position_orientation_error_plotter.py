#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
import tf.transformations as tft

# 全局变量，用于存储当前位置、姿态和期望位置、姿态
current_position = None
current_orientation = None

# 定义期望的位置和姿态
desired_position = Point(1.0, 0.0, 0.0)  # 期望位置
desired_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 期望姿态，使用四元数

# 定义用于计算欧拉角误差的函数
def quaternion_to_euler(q):
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)
    """
    return tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

def compute_orientation_error(current_q, desired_q):
    """
    计算姿态（欧拉角）的误差
    """
    current_euler = quaternion_to_euler(current_q)
    desired_euler = quaternion_to_euler(desired_q)
    
    error_roll = current_euler[0] - desired_euler[0]
    error_pitch = current_euler[1] - desired_euler[1]
    error_yaw = current_euler[2] - desired_euler[2]
    
    return error_roll, error_pitch, error_yaw

# 定义位置的回调函数
def position_callback(msg):
    global current_position
    current_position = msg

# 定义姿态（IMU）的回调函数
def orientation_callback(msg):
    global current_orientation
    current_orientation = msg.orientation

def plot_error():
    plt.ion()  # 开启交互模式
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 8))
    
    while not rospy.is_shutdown():
        if current_position is not None and current_orientation is not None:
            # 计算位置误差
            error_x = current_position.x - desired_position.x
            error_y = current_position.y - desired_position.y
            error_z = current_position.z - desired_position.z
            
            # 计算姿态误差
            error_roll, error_pitch, error_yaw = compute_orientation_error(current_orientation, desired_orientation)
            
            # 清除当前图像
            ax1.clear()
            ax2.clear()
            
            # 绘制位置误差
            ax1.bar(['X', 'Y', 'Z'], [error_x, error_y, error_z])
            ax1.set_ylim([-10, 10])
            ax1.set_title('Position Error')
            ax1.set_ylabel('Error (m)')
            
            # 绘制姿态误差
            ax2.bar(['Roll', 'Pitch', 'Yaw'], [error_roll, error_pitch, error_yaw])
            ax2.set_ylim([-np.pi, np.pi])
            ax2.set_title('Orientation Error (Euler)')
            ax2.set_ylabel('Error (rad)')
            
            # 显示误差值
            for i, v in enumerate([error_x, error_y, error_z]):
                ax1.text(i, v, f'{v:.2f}', ha='center', va='bottom')
            for i, v in enumerate([error_roll, error_pitch, error_yaw]):
                ax2.text(i, v, f'{v:.2f}', ha='center', va='bottom')
            
            # 绘制图像
            plt.pause(0.1)

if __name__ == '__main__':
    try:
        # 初始化ROS节点
        rospy.init_node('position_orientation_error_plotter', anonymous=True)
        
        # 订阅无人机的当前位置和姿态
        rospy.Subscriber('/drone/position', Point, position_callback)
        rospy.Subscriber('/drone/imu', Imu, orientation_callback)
        
        # 启动绘图
        plot_error()
        
        # 保持ROS运行
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
