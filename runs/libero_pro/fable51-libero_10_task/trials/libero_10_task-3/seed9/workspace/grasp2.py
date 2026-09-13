import numpy as np, sys
from rob import *
r = Robot("g2")
R1 = np.load("snaps/R_side.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.04; az=np.radians(60)
q0 = r.arm_q()
pre = np.array([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG])
for i in range(3):
    q = r.ik_tcp(pre, R1, seed=q0)
    if q is not None and np.abs(q-q0).max() < 0.5: break
r.move_q(q, 3.0); t,_ = r.tcp(); print("pre tcp", np.round(t,3))
if np.abs(t-pre).max() > 0.01:
    r.move_q(q, 3.0); t,_ = r.tcp(); print("pre tcp (resent)", np.round(t,3))
# advance along Z to grasp in 2 steps
path = cart_path(r, pre, R1, [BOT[0], BOT[1], ZG], R1, 3, q, max_step=0.4)
if path is None: raise SystemExit
r.move_q(path[-1], 4.0, via=path[:-1]); t,_ = r.tcp(); print("grasp tcp", np.round(t,3), "force", np.round(r.force(),2))
if np.abs(t-[BOT[0],BOT[1],ZG]).max() > 0.008:
    r.move_q(path[-1], 3.0); t,_ = r.tcp(); print("grasp tcp (resent)", np.round(t,3))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_grasp.png")
r.gripper(0.0)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
r.snap("sideview")
