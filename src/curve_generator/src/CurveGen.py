# a class to generate a curve based on a set of waypoints

import numpy as np
from geomdl import fitting
np.float = float
from geomdl.visualization import VisMPL
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class PointConverter:
    '''
    A class to extract the waypoints date from CoppeliaSim 

    '''
    def __init__(self, name='/sensor',):
        
        client = RemoteAPIClient()
        self.sim = client.require('sim')

        self.handle = self.sim.getObject(name)

    def updateData(self):
        # get the points in the camera frame
        self.data = np.array(self.sim.readVisionSensor(self.handle)[2][2:])\
            .reshape(-1,4)[:,0:3]
        # transform the points to the world frame
        self.transform = np.array(self.sim.getObjectMatrix(self.handle)).reshape(3,4)
        self.rot = self.transform[:,0:3]
        self.pos = self.transform[:,3].reshape(-1, 1)
        self.data = self.rot@self.data.T + self.pos
        self.data = self.data.T
        

    def getPoints(self,update=True):
        if update:
            self.updateData()
        return self.data.copy()

class CurveGen:
    def __init__(self, waypointConverter = PointConverter()):
        self.waypointConverter = waypointConverter

        # curve parameters
        self.degree_u = 3
        self.degree_v = 3
        self.size_u = 10
        self.size_v = 10

        # curve object
        self.curve = None
        self.updateCurve()

    def updateCurve(self):
        # get the waypoints
        points = self.waypointConverter.getPoints()
        points = points.tolist()
        # fit a b-spline curve to the waypoints
        self.curve = fitting.approximate_surface(points, self.size_u, self.size_v, \
                                                 self.degree_u, self.degree_v)
        
    def visualizeCurve(self,showCtrlPts=False):
        if showCtrlPts == False:
            vis_config = VisMPL.VisConfig(ctrlpts=False)
            self.curve.vis = VisMPL.VisSurface(config=vis_config)
        else:
            self.curve.vis = VisMPL.VisSurface()
        self.curve.render()
        

    def getPointPos(self, u, v):
        # evaluate the curve at the given u and v values
        return self.curve.evaluate_single((u, v))

    def getPointNormal(self, u, v):
        # evaluate the curve derivative at the given u and v values
        deriv = self.curve.derivatives(u, v, order=1)
        # get the tengent vector
        tangent_u = np.array(deriv[1][0])
        tangent_v = np.array(deriv[0][1]) 
        # get the normal vector
        normal = np.cross(tangent_u, tangent_v)
        if np.linalg.norm(normal) == 0:
            Exception("Normal vector is zero")
        normal = normal / np.linalg.norm(normal)
        return normal

    def getPoints(self, u,v):
        # get the position and normal of the point at the given u and v values
        pos = self.getPointPos(u,v)
        normal = self.getPointNormal(u,v)
        return pos, normal
    
    def convertCurve(self, U, V):
        pos_list = []
        normal_list = []
        for i in range(len(U)):
            pos, normal = self.getPoints(U[i], V[i])
            pos_list.append(pos)
            normal_list.append(normal)
        return pos_list, normal_list






