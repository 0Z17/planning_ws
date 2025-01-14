import open3d as o3d
import numpy as np

# 加载点云
pcd = o3d.io.read_point_cloud("data/pointcloud.pcd")
print("Original point cloud:")
print(pcd)

# 可视化原始点云
o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")

# 平移矩阵
translation = np.array([
    [1, 0, 0, 1],  # X 平移 1
    [0, 1, 0, 2],  # Y 平移 2
    [0, 0, 1, 3],  # Z 平移 3
    [0, 0, 0, 1]
])

# 旋转矩阵 (绕 Z 轴旋转 45 度)
theta = np.pi / 4  # 45 度
rotation = np.array([
    [np.cos(theta), -np.sin(theta), 0, 0],
    [np.sin(theta), np.cos(theta),  0, 0],
    [0,             0,              1, 0],
    [0,             0,              0, 1]
])

# 组合平移和旋转
transformation = np.dot(translation, rotation)

# 应用变换到点云
pcd.transform(transformation)

# 打印变换后的点云信息
print("Transformed point cloud:")
print(pcd)

# 可视化变换后的点云
o3d.visualization.draw_geometries([pcd], window_name="Transformed Point Cloud")

# 保存变换后的点云
o3d.io.write_point_cloud("data/pointcloud_trans.pcd", pcd)
