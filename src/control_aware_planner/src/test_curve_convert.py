import sys
from pathlib import Path
import rospkg
import rospy

rospkg = rospkg.RosPack()
package_path = rospkg.get_path('curve_generator')
sys.path.append(package_path + '/src')

from CurveGen import CurveGen
from invkin import IkSolver
from visCurve import vis_curve



if __name__ == '__main__':
    cg = CurveGen()
    iks = IkSolver()

    # get the curve data 
    cg.updateCurve()
    iks.update_curve(cg)

    # load the uv trajectory
    
    



                