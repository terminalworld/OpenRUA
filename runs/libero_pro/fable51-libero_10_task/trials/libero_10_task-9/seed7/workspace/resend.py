import sys
from rlib import *
r = Robot()
q = [float(x) for x in sys.argv[1].split(",")]
r.move_joints([q], float(sys.argv[2]) if len(sys.argv)>2 else 3.0)
print(np.round(r.joints(),3))
