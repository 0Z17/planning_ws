from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np

# the class to control the quadcopter:
class UAM:
    def __init__(self,name='/skyvortex',joint_name='/operator_1_joint'):

        # client = RemoteAPIClient("172.30.144.1")
        client = RemoteAPIClient("172.31.16.1")
        self.sim = client.require('sim')

        self.handle = self.sim.getObject(name)
        self.joint_handle = self.sim.getObject(joint_name)
        self.coll_collection = self.sim.createCollection()
        self.sim.addItemToCollection(self.coll_collection,self.sim.handle_tree,self.handle,0B00)


        # joint angle offset between the joint and the world frame
        self.joint_angle_offset = np.pi/6 
        # link length of the operator_link
        self.link_len = 0.95
    
    def set_position(self,position):
        self.sim.setObjectPosition(self.handle,position)

    def get_position(self):
        return self.sim.getObjectPosition(self.handle)
    
    def set_joint_position(self,position):
        """
        Set the joint position of the UAM in radians
        """
        # self.sim.setJointPosition(self.joint_handle, position - self.joint_angle_offset)
        self.sim.setJointTargetPosition(self.joint_handle, position - self.joint_angle_offset)
    
    def get_joint_position(self):
        return self.sim.getJointPosition(self.joint_handle) + self.joint_angle_offset
    
    def set_yaw(self,yaw):
        """
        Set the yaw of the UAM in radians
        """
        self.sim.setObjectOrientation(self.handle, [0,0,yaw])
    
    def get_yaw(self):
        return self.sim.setObjectOrientation(self.handle)
    
    def check_collision(self):
        """
        Check if the UAM is in collision with the environment \n
        True if in collision, False otherwise
        """
        return self.sim.checkCollision(self.coll_collection,self.sim.handle_all)[0]
    
    def set_state(self,state):
        """
        Set the state of the UAM
        """
        self.set_position(state[:3])
        self.set_yaw(state[3])
        self.set_joint_position(state[4])

if __name__ == '__main__':
    uam = UAM()
    print(uam.get_position())