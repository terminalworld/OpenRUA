import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("drag")
b = math.radians(15)
s, c = math.sin(b), math.cos(b)
R = np.column_stack([[0,-1,0],[-c,0,-s],[s,0,-c]])  # x_h, y_h, z_h
assert abs(np.linalg.det(R)-1) < 1e-6
q = R_quat(R)
def go(p, sec=3, via=None):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v, q) for v in via])
    if code is None: sys.exit("IK fail")
    return code
nx = -0.142
go([nx, 0.01, 1.02], 4)
r.report()
go([nx, 0.01, 0.915], 3)
r.report()
f = r.close()
if not (0.004 < f[0] < 0.012):
    print("grasp looks wrong", f); sys.exit(1)
# drag along -x in small steps
xs = np.arange(nx-0.025, -0.26, -0.025)
go([xs[-1], 0.01, 0.915], 5, via=[[x, 0.01, 0.915] for x in xs[:-1]])
r.report()
print("fingers after drag", r.fingers())
r.open()
go([xs[-1], 0.01, 1.02], 3)
r.report()
