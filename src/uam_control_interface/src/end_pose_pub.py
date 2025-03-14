#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped

class TfPosePublisher:
    def __init__(self):
        rospy.init_node('tf_pose_publisher', anonymous=True)
        
        # 创建tf2缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 创建位姿发布者
        self.pose_pub = rospy.Publisher('end_link_pose', PoseStamped, queue_size=10)
        
        # 设置发布频率（10Hz）
        self.rate = rospy.Rate(10)
        
        # 源坐标系和目标坐标系
        self.target_frame = "end_Link"
        self.source_frame = "map"

    def run(self):
        while not rospy.is_shutdown():
            try:
                # 获取最新的坐标变换
                transform = self.tf_buffer.lookup_transform(
                    self.source_frame,
                    self.target_frame,
                    rospy.Time(0),  # 获取最新可用变换
                    rospy.Duration(1.0)
                )
                
                # 创建PoseStamped消息
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = self.source_frame
                
                # 填充位置信息
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z
                
                # 填充方向信息
                pose_msg.pose.orientation = transform.transform.rotation
                
                # 发布位姿
                self.pose_pub.publish(pose_msg)
                
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("TF转换异常: %s", str(e))
                self.rate.sleep()
                continue
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = TfPosePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
