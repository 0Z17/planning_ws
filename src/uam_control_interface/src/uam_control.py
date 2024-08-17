
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget ,ActuatorControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
 
current_state = State()
 
def state_cb(msg):
    global current_state
    current_state = msg
 
local_pos = PoseStamped()
 
def local_pos_cb(local_pos_msg):
    global local_pos
    local_pos = local_pos_msg
 
class UamControl:

    def __init__(self):
        rospy.init_node("uam_control_py")

        self.current_state = State()
        self.local_pos = PoseStamped()
 
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.local_pos_cb)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        self.position_target_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.joint_pos_pub = rospy.Publisher("mavros/actuator_control", ActuatorControl, queue_size=10)

    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, local_pos_msg):
        self.local_pos = local_pos_msg

    def set_pose(self, x, y, z, yaw):
        pose = PositionTarget()
        pose.header.stamp = rospy.Time.now()
        pose.coordinate_frame = 1
        
        pose.type_mask = 0b101111111000 # only position
        # ignore mask : YAW_RATE | YAW | FORCE | AFZ | AFY | AFX | VZ | VY | VX | PZ | PY | PX
    
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.yaw = yaw

        self.position_target_pub.publish(pose)

    def set_vel(self, vx, vy, vz, yaw_rate):
        pose = PositionTarget()
        pose.header.stamp = rospy.Time.now()
        pose.coordinate_frame = 1

        pose.type_mask = 0b011111000111 # only velocity
        # ignore mask : YAW_RATE | YAW | FORCE | AFZ | AFY | AFX | VZ | VY | VX | PZ | PY | PX

        pose.velocity.x = vx
        pose.velocity.y = vy
        pose.velocity.z = vz

        pose.yaw_rate = yaw_rate

        self.position_target_pub.publish(pose)

    def set_acc(self, ax, ay, az):

        pose = PositionTarget()
        pose.header.stamp = rospy.Time.now()
        pose.coordinate_frame = 1

        pose.type_mask = 0b111000111111 # only acceleration
        # ignore mask : YAW_RATE | YAW | FORCE | AFZ | AFY | AFX | VZ | VY | VX | PZ | PY | PX

        pose.acceleration_or_force.x = ax
        pose.acceleration_or_force.y = ay
        pose.acceleration_or_force.z = az
    
        self.position_target_pub.publish(pose)

    def set_force(self, fx, fy, fz):

        pose = PositionTarget()
        pose.header.stamp = rospy.Time.now()
        pose.coordinate_frame = 1

        pose.type_mask = 0b110000111111 # only force
        # ignore mask : YAW_RATE | YAW | FORCE | AFZ | AFY | AFX | VZ | VY | VX | PZ | PY | PX

        pose.acceleration_or_force.x = fx
        pose.acceleration_or_force.y = fy
        pose.acceleration_or_force.z = fz

        self.position_target_pub.publish(pose)


