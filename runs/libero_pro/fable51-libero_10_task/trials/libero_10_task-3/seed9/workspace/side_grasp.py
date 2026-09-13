import numpy as np
from rob import *
r = Robot("sg")
BOT = np.array([-0.166, 0.073]); ZG = 1.04
R = np.stack([[0,0,-1],[0,1,0],[1,0,0]], 1).astype(float)  # cols X,Y,Z
r.gripper(0.04)
q0 = r.arm_q()
# pre-grasp: 7cm short of the bottle axis, same height
q1 = r.ik_tcp([BOT[0]-0.07, BOT[1], ZG], R, seed=q0)
print("pregrasp IK", None if q1 is None else np.round(q1,3))
if q1 is None: raise SystemExit
# also check grasp + lift IK before moving
q2 = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q1); print("grasp IK", None if q2 is None else np.round(q2,3))
q3 = r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=q2 if q2 is not None else q1); print("lift IK", None if q3 is None else np.round(q3,3))
if q2 is None or q3 is None: raise SystemExit
r.move_q(q1, 6.0); p,_=r.tcp(); print("tcp", np.round(p,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("sideview", "/workspace/snaps/side_pre.png")
