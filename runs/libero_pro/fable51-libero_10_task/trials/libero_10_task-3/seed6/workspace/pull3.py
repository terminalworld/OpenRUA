import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("pull3")
q = down_quat(-math.pi/2)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v,q) for v in via], **kw)
    if code is None: sys.exit("move failed")
    return code
r.open()
wx = -0.105; y0 = 0.13
go([wx, y0, 1.10], 4)
go([wx, y0, 0.966], 3)
r.report()
f = r.close()
if not (0.002 < f[0] < 0.012): print("pinch on wall failed", f); sys.exit(1)
ys = np.arange(y0-0.01, y0-0.055, -0.01)
go([wx, ys[-1], 0.966], 5, via=[[wx, y, 0.966] for y in ys[:-1]])
print("fingers", r.fingers()); r.report()
r.open()
go([wx, ys[-1], 1.15], 3)
go([-0.15, -0.05, 1.25], 3)
