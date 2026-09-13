import sys
from rlib import *
r = Robot("park")
x, y, z = map(float, sys.argv[1:4])
R = top_down_R(0.0)
q = r.ik([x, y, z], R)
r.move_q(q); r.report()
