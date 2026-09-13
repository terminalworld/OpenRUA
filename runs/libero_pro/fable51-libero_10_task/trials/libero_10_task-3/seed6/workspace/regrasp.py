import math, numpy as np, sys
from rb import Robot, down_quat
r = Robot("rg")
q = down_quat(-math.pi/2)
def go(p, qq=q, sec=3, **kw):
    code = r.move_pose(p, qq, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
r.open()
tcp = [-0.259, 0.093, 0.922]
go([tcp[0], tcp[1], 1.10], sec=4)
go(tcp)
f = r.close()
if not (0.014 < f[0] < 0.03): print("grasp wrong", f); sys.exit(1)
go([tcp[0], tcp[1], 1.15], sec=3)
print("fingers", r.fingers())
# rotate to yaw 0 (bottle along x)
for yaw in (-math.pi/4, 0.0):
    go([-0.20, 0.05, 1.15], down_quat(yaw), sec=4)
    print("fingers", np.round(r.fingers(),4))
r.report()
