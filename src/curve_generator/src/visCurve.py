import rospy
from geomdl import NURBS
import numpy as np
np.float = float
from geomdl.visualization import VisMPL
from geomdl import exchange
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def nurbs_to_triangles(surf):
    # Evaluate the surface
    surf.evaluate()
    points = np.array(surf.evalpts)
    
    triangles = []
    sample_size_u = surf.sample_size[0]
    sample_size_v = surf.sample_size[1]

    # Convert to triangles
    for i in range(sample_size_u - 1):
        for j in range(sample_size_v - 1):
            p1 = points[i * sample_size_v + j]
            p2 = points[(i + 1) * sample_size_v + j]
            p3 = points[i * sample_size_v + (j + 1)]
            p4 = points[(i + 1) * sample_size_v + (j + 1)]
            triangles.append([p1, p2, p3])
            triangles.append([p2, p4, p3])

    return triangles

def vis_curve(surf):

    triangles = nurbs_to_triangles(surf)

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "nurbs_surface"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    # 设置标记的颜色
    marker.color.a = 1.0  # 不透明度
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # 设置标记的点
    for tri in triangles:
        for vertex in tri:
            point = Point()
            point.x = vertex[0]
            point.y = vertex[1]
            point.z = vertex[2]
            marker.points.append(point)

    # 设置标记的尺寸
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # 发布标记
    pub.publish(marker)

    # rospy.loginfo("NURBS surface visualization sent to Rviz")