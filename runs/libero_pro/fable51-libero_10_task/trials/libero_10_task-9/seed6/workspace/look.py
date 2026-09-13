import sys, numpy as np
from rob import *; from ikbest import best_ik
r = Robot("look")
x,y,z = map(float, sys.argv[1:4]); sec = float(sys.argv[4]) if len(sys.argv)>4 else 4.0
print("now q", np.round(r.joints(),3))
R = R_from_axes([0,0,-1],[0,1,0])
q = best_ik(r, (x,y,z), R)
print("ik", None if q is None else np.round(q,3))
if q is None: sys.exit(1)
r.move_joints(q, sec)
p, RR = r.tcp(); print("tcp now", np.round(p,3))
