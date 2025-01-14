#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def publish_pcd():
    rospy.init_node('custom_pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/custom_topic_name', PointCloud2, queue_size=10)

    # 加载 .pcd 文件
    cloud = pcl.load('/path/to/your/file.pcd')

    # 转换为 PointCloud2 消息
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'new_frame_id'  # 设置自定义 frame_id

    points = []
    for point in cloud:
        points.append([point[0], point[1], point[2]])

    cloud_msg = pc2.create_cloud_xyz32(header, points)

    rate = rospy.Rate(10)  # 发布频率
    while not rospy.is_shutdown():
        cloud_msg.header.stamp = rospy.Time.now()  # 更新时间戳
        pub.publish(cloud_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pcd()
    except rospy.ROSInterruptException:
        pass
