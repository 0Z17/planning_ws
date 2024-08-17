#! /usr/bin/env python3
 
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
 
current_state = State()
in_task = False
 
def state_cb(msg):
    global current_state
    current_state = msg
 
local_pos = PoseStamped()
 
def local_pos_cb(local_pos_msg):
    global local_pos
    local_pos = local_pos_msg

def task_cb(msg):
    global in_task
    in_task = msg
 
if __name__ == "__main__":
    rospy.init_node("takeoff_py")
 
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_pos_cb)
    task_sub = rospy.Subscriber("uam_control_interface/in_task", Bool, callback = task_cb)
 
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
 
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
 
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
 
 
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(250)
 
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
 
    pose = PoseStamped()
 
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
 

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
 
        local_pos_pub.publish(pose)
        rate.sleep()
 
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
 
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
 
    last_req = rospy.Time.now()
    
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
 
        if not in_task:
            local_pos_pub.publish(pose)
            
        if(math.fabs(local_pos.pose.position.z - 2) <= 0.3) and not reach_height:
            reach_height = True
            rospy.loginfo("Reach height")
        
        rate.sleep()