"""Lower to z=1.05, then yaw 180 deg to approach (0,+.707,-.707) (insertion orientation)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
r = Planner("step7")
tcp, R0 = r.solve_fk(r.joints()); tcp = tcp + TCP*R0[:,2]
low = tcp.copy(); low[2] = 1.05
print("lower ok", r.move_line_tcp([low], R0, avoid=True))
tcp, R0 = r.solve_fk(r.joints()); tcp = tcp + TCP*R0[:,2]
appr = np.array([0, np.cos(np.radians(45)), -np.sin(np.radians(45))])
best = None
for fa in ([1,0,0], [-1,0,0]):
    R = R_from_axes(appr, fa)
    q = best_ik(r, tcp, R)
    if q is None: continue
    traj = r.plan_joints(q)
    if traj is None: continue
    L = sum(abs(np.array(traj.points[-1].positions) - np.array(traj.points[0].positions)))
    print("fa", fa, "q", np.round(q,3), "pathlen", round(L,3))
    if best is None or L < best[0]: best = (L, traj, q)
if best is None: sys.exit("no plan")
r.execute(best[1])
p, R = r.solve_fk(r.joints()); print("tcp after", np.round(p+TCP*R[:,2],3), "appr", np.round(R[:,2],3), "fa", np.round(R[:,1],3), "fingers", np.round(r.fingers(),4))
