import numpy as np, sys
from rob import *
r = Robot("look")
print("q", np.round(r.arm_q(),3), "gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
for c in sys.argv[1:]: r.snap(c)
