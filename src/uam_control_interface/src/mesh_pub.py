#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import time
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA

def mesh_publisher():
    rospy.init_node('mesh_display_node')
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    marker_pub = rospy.Publisher('/obstacles_markers', MarkerArray, queue_size=10)

    obstacles = [
        # [尺寸x,尺寸y,尺寸z, 位置x,位置y,位置z]
        [0.45, 0.1, 0.1, 0.3, 0.2, 1.1],
        [0.1, 0.1, 0.65, 0.2, -1.2, 2.23],
        [0.25, 0.45, 0.1, -0.11, -0.3, 1.33],
        [0.1, 1.3, 0.7, 1.6, 2.5, 2.33],
        [0.25, 1.05, 0.25, 0.4, 1.1, 3.8],
        [0.1, 0.1, 0.15, 0.3, 0.45, 1.8],
        [0.1, 0.45, 0.6, 0.8, 0.0, 3.45]
    ]
    
    marker = Marker()
    marker.header.frame_id = "floor"  # 设置坐标系
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "file:///home/wsl/firmware_ws/src/Firmware/Tools/sitl_gazebo/models/NURBS/meshes/NURBS_test.stl"
    # marker.mesh_resource = "file:///home/wsl/firmware_ws/src/Firmware/Tools/sitl_gazebo/models/WindTURBINE/meshes/wind_turbine.stl"
    marker.action = Marker.ADD
    
    # 设置姿态和缩放
    marker.pose.position.x = 1.500000
    # marker.pose.position.x = 10.000000
    marker.pose.position.y = 0.0
    # marker.pose.position.y = 10.0
    marker.pose.position.z = 0.0

    marker.pose.orientation.w = 0.7853981
    # marker.pose.orientation.w = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = -0.7853981
    # marker.pose.orientation.z = 0.0
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0  # 缩放比例
    
    # 设置颜色（RGBA，0-1）
    marker.color.r = 0.8
    marker.color.g = 0.8
    marker.color.b = 0.8
    marker.color.a = 1.0  # 不透明度

    time.sleep(5)
    pub.publish(marker)
    rospy.loginfo("Publishing mesh marker")
    

    rate = rospy.Rate(0.1)  
    # while not rospy.is_shutdown():
    # #     pub.publish(marker)
    # #     rospy.loginfo("Publishing mesh marker")
    # #     rate.sleep()

    marker_array = MarkerArray()

    for idx, (sx, sy, sz, px, py, pz) in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = "floor"  # 保持与URDF坐标系一致
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 设置位姿
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = pz
            marker.pose.orientation.w = 1.0  # 无旋转
            
            # 设置尺寸（与URDF中<size>对应）
            marker.scale = Vector3(sx, sy, sz)
            
            # 设置颜色（对应URDF中的ambient/diffuse）
            marker.color = ColorRGBA(0.5, 1.0, 0.5, 0.8)  # 半透明绿色
            
            # 设置持续显示
            marker.lifetime = rospy.Duration(0)  # 永久显示
            
            marker_array.markers.append(marker)

    rospy.loginfo("Publishing obstacle markers")
    # marker_pub.publish(marker_array)
        # rate.sleep()

if __name__ == '__main__':
    try:
        mesh_publisher()
    except rospy.ROSInterruptException:
        pass
