#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from my_topic.msg import error_msg
current_state = State()
pose_sub = PoseStamped()
def state_cb(msg):
    global current_state
    current_state = msg
def pos_cb(msg):
    global pose_sub
    pose_sub = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = pos_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    global_pub = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)
    # velocity_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    error_pub = rospy.Publisher("mytopic/error_msg", error_msg, queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(200)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose_pub = PoseStamped()
    

    twist = Twist()
    error =  error_msg()

    pose_pub.pose.position.x = 0.40
    pose_pub.pose.position.y = 0.34
    pose_pub.pose.position.z = 1.2

    # twist.linear.x = 0.05
    # twist.linear.y = 0.05
    # twist.linear.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose_pub)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    radius = 0.7
    speed = 0.5
    count = 0
    take_off = False
    reach_height = False

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                    take_off = True

                last_req = rospy.Time.now()
                
        if(take_off and count<=1000):
            time = rospy.Time.now().to_sec()
            pose_pub.pose.position.x = -0.048 + radius * math.sin(speed * time)
            pose_pub.pose.position.y = 1.03 + radius * math.cos(speed * time)
            

            x = pose_sub.pose.position.x
            y = pose_sub.pose.position.y
            z = pose_sub.pose.position.z
            error.error_x= -0.048+ radius * math.sin(speed * time) -x
            error.error_y= 1.03 + radius * math.sin(speed * time) -y
            error.error_z= 1.2  -z
            error.error_roll=0
            error.error_pitch=0
            error.error_yaw=0

            error_pub.publish(error)
            local_pos_pub.publish(pose_pub)
            
            # velocity_pub.publish(twist)
        elif(take_off):
            land_set_mode = SetModeRequest()
            land_set_mode.custom_mode = 'AUTO.LAND'
            if(set_mode_client.call(land_set_mode).mode_sent == True):
                rospy.loginfo("Land enabled")
        elif(not take_off):
            local_pos_pub.publish(pose_pub)

        rate.sleep()
