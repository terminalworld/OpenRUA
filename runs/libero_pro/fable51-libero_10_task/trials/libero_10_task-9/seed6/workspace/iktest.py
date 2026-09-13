import numpy as np
from rob import *
r = Robot("iktest")
q0 = r.joints(); print("q0", np.round(q0,3))
pos, R = r.solve_fk(q0); print("hand world", np.round(pos,3), "tcp", np.round(pos+TCP*R[:,2],3)); print(np.round(R,2))
tests = {
 "pregrasp(-y approach)": ((-0.080, -0.12, 0.945), R_from_axes([0,-1,0],[1,0,0])),
 "grasp(-y approach)":    ((-0.080, -0.207, 0.945), R_from_axes([0,-1,0],[1,0,0])),
 "lift":                  ((-0.080, -0.207, 1.15), R_from_axes([0,-1,0],[1,0,0])),
 "insert(+y approach)":   ((-0.035, 0.285, 1.02), R_from_axes([0,1,0],[1,0,0])),
 "pre-insert":            ((-0.035, 0.10, 1.02), R_from_axes([0,1,0],[1,0,0])),
 "topdown over mug":      ((-0.080, -0.281, 1.10), R_from_axes([0,0,-1],[0,1,0])),
}
for name,(tcp,R) in tests.items():
    for fa in ([1,0,0],[-1,0,0]) if "topdown" not in name else ([0,1,0],[1,0,0]):
        RR = R_from_axes(R[:,2], fa)
        q = r.solve_ik(hand_pose_from_tcp(tcp, RR), RR, seed=q0)
        print(f"{name} finger_axis={fa}: {'OK '+str(np.round(q,3)) if q else 'FAIL'}")
