import numpy as np, sys
from rob import *
r = Robot("j7")
q = r.joints(); q[6] = float(sys.argv[1])
r.move_joints(q, float(sys.argv[2]))
print("j7 now", round(r.joints()[6],3))
