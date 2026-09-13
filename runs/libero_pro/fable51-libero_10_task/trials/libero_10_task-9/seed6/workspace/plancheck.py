import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("plancheck")
tcp, R = r.tcp(); print("tcp", tcp.round(3))
wps = [tcp + [0,0,0.07], [-0.035, 0.12, tcp[2]+0.07], [-0.035, 0.12, 1.016], [-0.035, 0.286, 1.016], [-0.035, 0.286, 1.0]]
prev = tcp
for w in wps:
    traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R)], avoid=True, step=0.01)
    print("segment to", np.round(w,3), "fraction", frac)
    # note: cartesian() plans from the CURRENT robot state, so segments after the first are only indicative
    break
# plan whole path at once
traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R) for w in wps], avoid=True, step=0.01)
print("whole path fraction", frac)
