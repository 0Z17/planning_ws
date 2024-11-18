#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

local_pos = PoseStamped()

# 订阅无人机的当前位置
def local_pos_cb(local_pos_msg):
    global local_pos
    local_pos = local_pos_msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sb = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_pos_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    position = PoseStamped()

    position.pose.position.x = 0
    position.pose.position.y = 0
    position.pose.position.z = 2
    
    pose = PositionTarget()

    pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    pose.header.frame_id = "Drone"
    pose.header.stamp    = rospy.Time.now()
    pose.type_mask = (
                      PositionTarget.IGNORE_PX  |
                      PositionTarget.IGNORE_PY  |
                      PositionTarget.IGNORE_PZ  |
                      PositionTarget.IGNORE_VX  |
                      PositionTarget.IGNORE_VY  |
                      PositionTarget.IGNORE_VZ  |
                    #   PositionTarget.IGNORE_AFX |
                    #   PositionTarget.IGNORE_AFY |
                    #   PositionTarget.IGNORE_AFZ |
                    #   PositionTarget.FORCE      |
                      PositionTarget.IGNORE_YAW |
                      PositionTarget.IGNORE_YAW_RATE)
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 2
    pose.velocity.x = 0.1
    pose.velocity.y = 1
    pose.velocity.z = 1.5
    pose.acceleration_or_force.x = 0
    pose.acceleration_or_force.y = 0
    pose.acceleration_or_force.z = 0.7
    pose.yaw = 0
    pose.yaw_rate = 0


    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(position)
        local_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    count = 0
    k = 0.2
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
        
                
        if(count == 0 and not reach_height):
            local_pos_pub.publish(position)
            if(math.fabs(local_pos.pose.position.z - 2) <= 0.1):
                reach_height = True
                rospy.loginfo("Reach Height!")
        elif(reach_height and count < 999):
            local_pub.publish(pose)
            count += 1
        elif(count >= 999):
            land_set_mode = SetModeRequest()
            land_set_mode.custom_mode = 'AUTO.LAND'
            if(set_mode_client.call(land_set_mode).mode_sent == True):
                rospy.loginfo("Land enabled")

        rate.sleep()