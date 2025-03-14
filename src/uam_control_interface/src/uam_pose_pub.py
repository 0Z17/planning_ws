#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PathPublisher:
    def __init__(self):
        rospy.init_node('tf_pose_publisher', anonymous=True)
        
        # 创建tf2缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.end_path_msg = Path()
        self.base_path_msg = Path()
        self.marker_array = MarkerArray()
        self.marker_count = 0
        self.arrow_length = 0.4
        self.interval_count = 0
        self.interval = 150

        # 创建位姿发布者
        self.end_path_pub = rospy.Publisher('/end_path', Path, queue_size=10)
        self.base_path_pub = rospy.Publisher('/base_path', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

        self.conctact_trigger_sub = rospy.Subscriber('/contact_trigger', Bool, self.contact_trigger_callback)
        self.conctact_trigger = False 
        
        # 设置发布频率（10Hz）
        self.rate = rospy.Rate(5)
        
        # 源坐标系和目标坐标系
        self.end_target_frame = "end_Link"
        self.base_target_frame = "base_link"
        self.source_frame = "floor"

        self.z_offset = 0.4

    def run(self):
        while not rospy.is_shutdown():
            if not self.conctact_trigger:
                self.rate.sleep()
                continue
            try:
                # 获取最新的坐标变换
                end_transform = self.tf_buffer.lookup_transform(
                    self.source_frame,
                    self.end_target_frame,
                    rospy.Time(0),  # 获取最新可用变换
                    rospy.Duration(1.0)
                )

                base_transform = self.tf_buffer.lookup_transform(
                    self.source_frame,
                    self.base_target_frame,
                    rospy.Time(0),  # 获取最新可用变换
                    rospy.Duration(1.0)
                )
                
                # 创建PoseStamped消息
                end_pose_msg = PoseStamped()
                end_pose_msg.header.stamp = rospy.Time.now()
                end_pose_msg.header.frame_id = self.source_frame
                
                # 填充位置信息
                end_pose_msg.pose.position.x = end_transform.transform.translation.x
                end_pose_msg.pose.position.y = end_transform.transform.translation.y
                end_pose_msg.pose.position.z = end_transform.transform.translation.z + self.z_offset
                
                # 填充方向信息
                end_pose_msg.pose.orientation = end_transform.transform.rotation
                
                # 发布路径
                self.end_path_msg.header.stamp = rospy.Time.now()
                self.end_path_msg.header.frame_id = self.source_frame
                self.end_path_msg.poses.append(end_pose_msg)


                self.end_path_pub.publish(self.end_path_msg)


                # 创建PoseStamped消息
                base_pose_msg = PoseStamped()
                base_pose_msg.header.stamp = rospy.Time.now()
                base_pose_msg.header.frame_id = self.source_frame

                # 填充位置信息
                base_pose_msg.pose.position.x = base_transform.transform.translation.x
                base_pose_msg.pose.position.y = base_transform.transform.translation.y
                base_pose_msg.pose.position.z = base_transform.transform.translation.z + self.z_offset
                
                # 填充方向信息
                base_pose_msg.pose.orientation = base_transform.transform.rotation
                
                # 发布路径
                self.base_path_msg.header.stamp = rospy.Time.now()
                self.base_path_msg.header.frame_id = self.source_frame
                self.base_path_msg.poses.append(base_pose_msg)

                self.base_path_pub.publish(self.base_path_msg)

                if not (self.interval_count % self.interval == 0):
                    self.interval_count += 1
                    continue
                self.interval_count = 1

                # publish marker
                marker = Marker()
                marker.header.frame_id = self.source_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "normal"
                marker.id = self.marker_count
                self.marker_count += 1
                marker.type = Marker.ARROW
                marker.action = Marker.ADD

                start_point = Point()
                start_point.x = end_transform.transform.translation.x
                start_point.y = end_transform.transform.translation.y
                start_point.z = end_transform.transform.translation.z

                end_point = Point()
                end_point.x = start_point.x + (base_transform.transform.translation.x - end_transform.transform.translation.x) * self.arrow_length
                end_point.y = start_point.y + (base_transform.transform.translation.y - end_transform.transform.translation.y) * self.arrow_length
                end_point.z = start_point.z + (base_transform.transform.translation.z - end_transform.transform.translation.z) * self.arrow_length

                marker.points.append(start_point)
                marker.points.append(end_point)
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.2

                self.marker_array.markers.append(marker)

                self.marker_pub.publish(self.marker_array)
                
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("TF转换异常: %s", str(e))
                self.rate.sleep()
                continue


            rospy.loginfo("publish path")
            
            self.rate.sleep()

    def contact_trigger_callback(self, msg):
        self.conctact_trigger = msg.data

if __name__ == '__main__':
    try:
        publisher = PathPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
