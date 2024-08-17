import rospy
from uam_control import UamControl
from std_msgs.msg import Bool
import os
import numpy as np

if __name__ == '__main__':
    uc = UamControl()

    task_pub = rospy.Publisher('/uam_control_interface/in_task', Bool, queue_size=10)

    traj = np.load(os.path.join(os.path.dirname(__file__), '../input/rrt_vel.npy'))

    count = 0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and count < traj.shape[0]:
        # uc.set_vel(0, 0, 2, 0)
        # uc.set_acc(0.1, 0.1, 1)
        # uc.set_force(0, 0, 0.1)

        vx = traj[count, 0]
        vy = traj[count, 1]
        vz = traj[count, 2]

        uc.set_vel(vx, vy, vz, 0)
        task_pub.publish(True)
        count += 1

        rate.sleep()
