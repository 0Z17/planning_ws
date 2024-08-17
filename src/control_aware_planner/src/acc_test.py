#! /usr/bin/env python
 
import rospy
import math
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseStamped, TwistStamped
from mavros_msgs.msg import State, ActuatorControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
 
current_state = State()
 
def state_cb(msg):
    global current_state
    current_state = msg
 
local_pos = PoseStamped()
 
def local_pos_cb(local_pos_msg):
    global local_pos
    local_pos = local_pos_msg
 
if __name__ == "__main__":
    rospy.init_node("offb_force_py")
 
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sb = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = local_pos_cb)
 
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    ActuatorControl_pub = rospy.Publisher("mavros/actuator_control", ActuatorControl, queue_size=10)

    acc_pub = rospy.Publisher("mavros/setpoint_accel/accel", Vector3Stamped, queue_size=10)
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", Vector3Stamped, queue_size=10)
    pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    def set_acc(x,y,z):
        acc = Vector3Stamped()
        acc.header.stamp = rospy.Time.now()
        acc.vector.x = 0.0
        acc.vector.y = 0.0
        acc.vector.z = 0.0
        acc_pub.publish(acc)

    def set_pos(x,y,z):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pos_pub.publish(pose)

    def set_vel(x,y,z):
        vel = Vector3Stamped()
        vel.header.stamp = rospy.Time.now()
        vel.vector.x = x
        vel.vector.y = y
        vel.vector.z = z
        vel_pub.publish(vel)
 
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
 
    actuator_control = ActuatorControl()
 
    actuator_control.group_mix = 0
    actuator_control.controls[0] = 0.000          # ROLL
    actuator_control.controls[1] = 0.000          # PITCH
    actuator_control.controls[2] = 0.000          # YAW
    actuator_control.controls[3] = 0.824        # THROTTLE
    actuator_control.controls[4] = 0.0          # FLAPS
    actuator_control.controls[5] = 0.0          # SPOILERS
    actuator_control.controls[6] = 0.0          # AIRBRAKES
    actuator_control.controls[7] = -1.0          # LANDING_GEAR
    actuator_control.controls[8] = 0.000         # X_THRUST
    actuator_control.controls[9] = -0.00          # Y_THRUST
    actuator_control.controls[10] = -0.824        # Z_THRUST
    # actuator_control.header.seq = 1
    # actuator_control.header.stamp = rospy.Time.now()
    # actuator_control.header.frame_id = "test"
 
    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
 
        local_pos_pub.publish(pose)
        ActuatorControl_pub.publish(actuator_control)
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
 
        if(count == 0 and not reach_height):
            local_pos_pub.publish(pose)
            if(math.fabs(local_pos.pose.position.z - 2) <= 0.3):
                reach_height = True
                rospy.loginfo("Reach Height!")
                count = 1
        elif(take_off and count == 1):
            # actuator_control.header.stamp = rospy.Time.now()        
            # ActuatorControl_pub.publish(actuator_control)
            set_acc(0.1, 0.0, -0.8)
            # set_vel(1.0, 0.0, 10.0)
        
 
        rate.sleep()