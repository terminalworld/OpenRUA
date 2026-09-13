"""Right the hanging mug: lower until base edge ~2mm above table, then move pinch on an arc about the base edge."""
import sys, numpy as np
from rob import *; from plan import Planner
a_len = float(sys.argv[1])      # pinch -> base plane distance along axis (m)
axis_z = float(sys.argv[2])     # current mug axis height
TABLE = 0.9017; R_MUG = 0.0465
r = Planner("right")
tcp, R = r.tcp(); x0, y0, z0 = tcp
E = np.array([x0, y0 - a_len, TABLE + 0.002])       # pivot: base bottom edge (after lowering)
drop = (axis_z - R_MUG) - E[2]
print("lower by", round(drop,4))
ok = r.move_line_tcp([tcp - [0, 0, drop]], R, avoid=False); print("lower ok", ok)
p1 = r.tcp()[0]; print("after lower tcp", p1.round(4), "fingers", np.round(r.fingers(),4))
rel = p1 - E; rad = np.hypot(rel[1], rel[2]); th0 = np.arctan2(rel[2], rel[1])
print("radius", round(rad,4), "theta0", round(np.degrees(th0),1))
pts = []
for k in range(1, 19):
    th = th0 + np.radians(90) * k / 18
    pts.append(E + [0, rad*np.cos(th), rad*np.sin(th)])
print("final pinch", pts[-1].round(4))
ok = r.move_line_tcp(pts, R, avoid=False, min_fraction=0.95); print("arc ok", ok)
p2 = r.tcp()[0]; print("after arc tcp", p2.round(4), "fingers", np.round(r.fingers(),4))
