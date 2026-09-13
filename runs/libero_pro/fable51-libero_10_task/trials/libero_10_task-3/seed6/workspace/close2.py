import math, numpy as np, sys
from rb import Robot, R_quat
r = Robot("close2")
g = math.radians(30)
zh = np.array([0, math.sin(g), -math.cos(g)]); yh = np.array([-1.0, 0, 0]); xh = np.cross(yh, zh)
q = R_quat(np.column_stack([xh, yh, zh]))
def go(p, sec=3, **kw):
    code = r.move_pose(p, q, seconds=sec, at_tcp=True, **kw)
    if code is None: sys.exit("move failed")
    return code
r.close()
go([0.0, 0.14, 1.12], 5)
go([0.0, 0.14, 0.937], 3)
print("wrench", r.wrench().round(1))
for y in (0.165, 0.175, 0.185):
    c = go([0.0, y, 0.937], 3)
    w = r.wrench(); print(f"y {y}: code {c} wrench {w.round(1)}")
    if c != 0 or abs(w[1]) > 15: break
r.report()
go([0.0, 0.12, 1.15], 3)
go([-0.10, -0.05, 1.25], 3)
