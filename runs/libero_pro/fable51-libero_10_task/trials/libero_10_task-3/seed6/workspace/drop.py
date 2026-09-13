import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("drop")
q = down_quat(0.0)
def go(p, sec=3, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
go([0.042, 0.125, 1.20], 5)
print("fingers", np.round(r.fingers(),4))
go([0.042, 0.125, 0.99], 4)
p,_ = r.report()
print("wrench", r.wrench().round(1))
r.open()
go([0.042, 0.125, 1.20], 3)
go([-0.10, -0.05, 1.25], 3)
