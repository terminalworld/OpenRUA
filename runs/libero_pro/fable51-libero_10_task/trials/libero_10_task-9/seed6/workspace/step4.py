"""Step 4: planned transit to the pre-insert pose in front of the microwave (hand yawed 180 deg, pitch kept)."""
import numpy as np, sys
from plan import *; from ikbest import best_ik
r = Planner("step4")
tcp = np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv) >= 4 else np.array([-0.035, 0.12, 1.10])
approach = np.array([0, np.cos(np.radians(45)), -np.sin(np.radians(45))])   # toward +y and down
best = None
for fa in ([-1,0,0],[1,0,0]):
    R = R_from_axes(approach, fa)
    q = best_ik(r, tcp, R)
    print("finger axis", fa, "ik", None if q is None else np.round(q,3))
    if q is None: continue
    traj = r.plan_joints(q)
    if traj is None: continue
    pts = retime(traj); dur = pts[-1][1]
    print("  duration", round(dur,1))
    if best is None or dur < best[1]: best = (traj, dur)
if best is None: sys.exit("no plan")
r.execute(best[0])
p, RR = r.tcp(); print("tcp now", np.round(p,3), "approach", np.round(RR[:,2],3), "fingers", r.fingers())
