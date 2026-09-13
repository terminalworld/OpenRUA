import numpy as np, sys
from rob import *
r = Robot("tr")
R1 = np.load("snaps/R_side.npy"); q1, qg, ql = np.load("snaps/q_chain.npy")
q0 = r.arm_q(); tcp0, R0 = r.tcp()
tcp1, _ = r.tcp(q1)
print("from", np.round(tcp0,3), "to", np.round(tcp1,3))
# stage A: rise to z=1.2 keeping orientation
a = cart_path(r, tcp0, R0, [tcp0[0], tcp0[1], 1.2], R0, 2, q0)
# stage B: to above target at 1.2 with target orientation
b = cart_path(r, [tcp0[0], tcp0[1], 1.2], R0, [tcp1[0], tcp1[1], 1.2], R1, 8, a[-1]) if a else None
# stage C: descend to pre-grasp
c = cart_path(r, [tcp1[0], tcp1[1], 1.2], R1, tcp1, R1, 3, b[-1]) if b else None
if not (a and b and c): raise SystemExit("path failed")
path = a + b + c
print("path joint steps:", [round(float(np.abs(path[i]-(path[i-1] if i else q0)).max()),2) for i in range(len(path))])
print("final vs q1 dist", round(np.abs(path[-1]-q1).max(),2))
if "--go" not in sys.argv: raise SystemExit("dry run")
r.gripper(0.04)
r.move_q(path[-1], 3.0*len(path), via=path[:-1])
t,_ = r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("agentview")
