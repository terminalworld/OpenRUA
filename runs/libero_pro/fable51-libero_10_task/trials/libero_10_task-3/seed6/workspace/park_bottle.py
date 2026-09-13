import math, numpy as np, sys
from rb import Robot, R_quat, down_quat
r = Robot("park")
q = down_quat(-math.pi/2)
def go(p, qq=q, sec=3, via=None, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    return code
go([-0.25, 0.10, 1.10], sec=4)
go([-0.25, 0.10, 0.94], sec=3)
r.open()
go([-0.25, 0.10, 1.10], sec=3)
r.report()
