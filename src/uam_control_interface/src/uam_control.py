
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget 
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetJointProperties
from std_msgs.msg import Float32

 
class UamControl:

    def __init__(self):
        rospy.init_node("uam_control_py")

        self.joint_name = "operator_1_joint"
        self.vehicle_name = "skyvortex"
        
        self.current_state = State()
        self.local_pos = PoseStamped()
 
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self.local_pos_cb)
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

        self.position_target_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.joint_pos_pub = rospy.Publisher(self.vehicle_name + "/" + self.joint_name + "/pos_cmd", Float32, queue_size=10)

        # initialize the joint service
        rospy.wait_for_service("gazebo/get_joint_properties")
        self.joint_client = rospy.ServiceProxy("gazebo/get_joint_properties", GetJointProperties)

        # param relate to the trans controler
        self.Kp = 0.01
        self.Kd = 0.01
        self.Ki = 0.001

        self.integral_error = 0.0
        self.prev_error = 0.0

        # param relate to the yaw controler
        self.Kp_yaw = 0.01
        self.Kd_yaw = 0.01
        self.Ki_yaw = 0.001

        self.integral_error_yaw = 0.0
        self.prev_error_yaw = 0.0

        # the system offset
        self.pose_offset = [0.0, 0.0, 0.0]
        self.joint_offset = np.pi/6

    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, local_pos_msg):
        self.local_pos = local_pos_msg

    def set_pose(self, x, y, z, yaw):
        """
        Set the position and yaw (in radians) of the UAV
        """
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

    def set_joint_pos(self, joint_pos):
        """
        set the joint position (in radians)
        """
        joint_pos_msg = Float32()
        joint_pos_msg.data = joint_pos - self.joint_offset
        self.joint_pos_pub.publish(joint_pos_msg)

    def get_joint_pos(self):
        """
        get the joint position (in radians)
        """
        response = self.joint_client(self.joint_name)
        joint_pos = response.position[0] + self.joint_offset
        return joint_pos

    def get_current_config(self):
        """
        move to the given state from the current state
        """
        # get the current position and yaw
        current_x = self.local_pos.pose.position.x + self.pose_offset[0]
        current_y = self.local_pos.pose.position.y + self.pose_offset[1]
        current_z = self.local_pos.pose.position.z + self.pose_offset[2]
        current_orientation = self.local_pos.pose.orientation
        current_yaw = euler_from_quaternion([current_orientation.w, current_orientation.x, current_orientation.y, current_orientation.z])[2]
        
        # current_joint_pos =self.get_joint_pos()

        current_position = np.array([current_x, current_y, current_z])

        return current_position, current_yaw, None #current_joint_pos
    
    def trans_contrl(self, dir_p, dir_v, dt):
        """
        control the volecity via pid control
        """
        pos = self.get_current_config()
        
        # Proportional term
        err_p = dir_p - pos
        P_out = self.Kp * err_p

        # Integral term
        self.integral_error += err_p * dt
        I_out = self.Ki * self.integral_error

        # Derivative term
        err_d = (err_p - self.prev_error) / dt
        D_out = self.Kd * err_d

        # combile all terms
        # vel = dir_v + P_out + I_out + D_out
        vel = dir_v + P_out + D_out

        # update previous error
        self.prev_error = err_p

        return vel

    def yaw_contrl(self, dir_p_yaw, dir_v_yaw, dt):
        """
        control the yaw via pid control
        """
        _, yaw, _ = self.get_current_config()

        # Proportional term
        err_p = dir_p_yaw - yaw
        P_out = self.Kp * err_p

        # Integral term
        self.integral_error_yaw += err_p * dt
        I_out = self.Ki * self.integral_error

        # Derivative term
        err_d = (err_p - self.prev_error) / dt
        D_out = self.Kd * err_d

        # combile all terms
        # vel = dir_v + P_out + I_out + D_out
        vel = dir_v_yaw + P_out + D_out

        return vel


    





