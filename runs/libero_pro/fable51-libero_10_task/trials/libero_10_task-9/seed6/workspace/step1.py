"""Step 1: go to the pre-grasp pose beside the white mug handle (approach from +y, pitched 45 deg down)."""
import numpy as np, sys
from plan import *; from ikbest import best_ik
r = Planner("step1")
pitch = np.radians(float(sys.argv[1]) if len(sys.argv)>1 else 45)
approach = np.array([0, -np.cos(pitch), -np.sin(pitch)])   # toward -y and down
R = R_from_axes(approach, [1,0,0])
bar = np.array([-0.080, -0.207, 0.947])                     # handle outer bar centre (world)
pre = bar - 0.10*approach                                   # 10 cm back along the approach
print("pre-grasp tcp", np.round(pre,3), "hand", np.round(hand_pose_from_tcp(pre,R),3))
q = best_ik(r, pre, R)
print("ik", None if q is None else np.round(q,3))
if q is None: sys.exit(1)
ok = r.goto_joints(q)
p, RR = r.tcp(); print("tcp now", np.round(p,3), "approach now", np.round(RR[:,2],3), "ok", ok)
