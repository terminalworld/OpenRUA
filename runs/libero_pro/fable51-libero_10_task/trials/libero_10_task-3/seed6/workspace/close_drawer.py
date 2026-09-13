import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("close")
g = math.radians(30)
zh = np.array([0, math.sin(g), -math.cos(g)]); yh = np.array([-1.0, 0, 0]); xh = np.cross(yh, zh)
R = np.column_stack([xh, yh, zh]); assert abs(np.linalg.det(R)-1) < 1e-6
q = R_quat(R)
def go(p, sec=3, via=None, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, via=None if via is None else [(v, q) for v in via], **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.0, 0.02, 1.12], 5)
go([0.0, 0.02, 0.937], 3)
r.report(); print("wrench", r.wrench().round(1))
ys = np.arange(0.04, 0.18, 0.02)
code = go([0.0, ys[-1], 0.937], 8, via=[[0.0, y, 0.937] for y in ys[:-1]])
print("push code", code, "wrench", r.wrench().round(1)); r.report()
go([0.0, 0.10, 1.15], 3)
go([-0.10, -0.05, 1.25], 3)
