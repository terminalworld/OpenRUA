import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("pull")
q = down_quat(-math.pi/2)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=via, **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.003, 0.025, 1.10], 4)
go([0.003, 0.025, 0.935], 3)
r.report()
print("wrench before", r.wrench().round(1))
ys = np.arange(0.015, -0.03, -0.01)
go([0.003, ys[-1], 0.935], 5, via=[[0.003, y, 0.935] for y in ys[:-1]])
print("wrench after", r.wrench().round(1))
r.report()
go([0.003, ys[-1], 1.10], 3)
