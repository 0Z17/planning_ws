import pinocchio as pin
import numpy as np
import os

base_path = os.path.dirname(__file__) + '/../../../'

urdf_filename = base_path + '/urdf/skyvortex.urdf'
model = pin.buildModelFromUrdf(urdf_filename)
# sdf_filename = base_path + '/sdf/skyvortex/skyvortex.sdf'
# model = pin.buildModelFromSdf(sdf_filename)
# model = pin.buildSampleModelManipulator()
print(model)

# add a floating base to the model
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

# 假设你已经有一个从URDF加载的模型
# urdf_path = "path_to_your_urdf.urdf"
# model = RobotWrapper.BuildFromURDF(urdf_path).model

# 创建一个新的空模型
new_model = pin.Model()

# 添加一个飞行关节作为根关节
joint_id = new_model.addJoint(
    new_model.getJointId("universe"),
    pin.JointModelFreeFlyer(),
    pin.SE3.Identity(),
    "flying_joint"
)

# 将原始模型中的所有关节和连杆添加到新的模型中
for j in range(1, len(model.joints)):
    parent_joint_id = model.parents[j]
    joint_placement = model.jointPlacements[j]
    new_model.addJoint(
        joint_id if parent_joint_id == 1 else new_model.getJointId(model.names[parent_joint_id]),
        model.joints[j],
        joint_placement,
        model.names[j]
    )

    # 添加刚体
    new_model.appendBodyToJoint(new_model.getJointId(model.names[j]), pin.Inertia.Zero(), pin.SE3.Identity())

# 为新的模型创建数据
new_data = new_model.createData()


