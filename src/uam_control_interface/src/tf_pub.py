#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf.transformations import quaternion_from_euler

class MavrosTfPublisher:
    def __init__(self):
        # initialize node
        rospy.init_node('mavros_tf_publisher', anonymous=True)

        # get parameters
        self.parent_frame = rospy.get_param('~parent_frame', 'map')
        self.child_frame = rospy.get_param('~child_frame', 'base_link')

        # create tf broadcaster
        self.br = tf.TransformBroadcaster()

        # subscribe to pose topic
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)

        rospy.loginfo("mavros_tf_publisher node started, parent frame: %s, child frame: %s", self.parent_frame, self.child_frame)

        # parameters
        self.pos_offset = rospy.get_param('~pos_offset', [0.0, 0.0, 0.0])
        # self.pos_offset = rospy.get_param('~pos_offset', [2.0, -1.5, 0.0])

        self.static_transform()  # publish static transform

    def pose_callback(self, msg):
        # get pose data
        position = msg.pose.position
        orientation = msg.pose.orientation

        # get time
        current_time = rospy.Time.now()

        # create transform message and send it to tf broadcaster
        self.br.sendTransform(
            (position.x, position.y, position.z),
            (orientation.x, orientation.y, orientation.z, orientation.w),
            current_time,
            self.child_frame,
            self.parent_frame
        )

    def run(self):
        rospy.spin()

    def static_transform(self):
        static_broadcaster = StaticTransformBroadcaster()
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "floor"  # 父坐标系
        static_transform.child_frame_id = "map"     # 子坐标系

        static_transform.transform.translation.x = -2.0
        static_transform.transform.translation.y = 1.5
        static_transform.transform.translation.z = 0.4

        quat = quaternion_from_euler(0, 0, 0)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        static_broadcaster.sendTransform(static_transform)
        rospy.loginfo("Static transform published: floor -> map")


if __name__ == '__main__':
    try:
        publisher = MavrosTfPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
