import rospkg
import sys

sys.path.append(rospkg.RosPack().get_path('curve_generator') + '/src')

from CurveGen import CurveGen

data_path = rospkg.RosPack().get_path('planning_utils') +'/data'
curve_name = "curve_02"

cg = CurveGen()
cg.visualizeCurve()
cg.saveCurve(data_path + "/" + curve_name + "/" + curve_name + ".json")
