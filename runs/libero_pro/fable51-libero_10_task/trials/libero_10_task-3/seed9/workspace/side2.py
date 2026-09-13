import numpy as np, sys
from rob import *
r = Robot("sg2")
BOT = np.array([-0.166, 0.073]); ZG = 1.04
az = np.radians(60)
Z = np.array([np.cos(az), np.sin(az), 0.0]); Y = np.array([-np.sin(az), np.cos(az), 0.0]); X = np.cross(Y, Z)
R = np.stack([X, Y, Z], 1)
np.save("snaps/R_side.npy", R)
seed = np.array([-0.66, 0.49, 0.21, -2.46, 1.6, 1.68, 0.98])
q0 = r.arm_q()
pre = np.array([BOT[0]-0.07*np.cos(az), BOT[1]-0.07*np.sin(az), ZG])
q1 = r.ik_tcp(pre, R, seed=seed); print("q1", np.round(q1,3), "dist from now", round(np.abs(q1-q0).max(),2))
qg = r.ik_tcp([BOT[0], BOT[1], ZG], R, seed=q1); ql = r.ik_tcp([BOT[0], BOT[1], ZG+0.11], R, seed=qg)
print("qg", np.round(qg,3)); print("ql", np.round(ql,3))
np.save("snaps/q_chain.npy", np.stack([q1,qg,ql]))
if "--go" not in sys.argv: raise SystemExit("dry run")
r.gripper(0.04)
# go via an intermediate high pose to avoid sweeping low: first raise tcp to 1.25 at current xy with current R
pos, Rc = r.fk_hand()
r.move_q(q1, 8.0)
t,_ = r.tcp(); print("tcp", np.round(t,3), "force", np.round(r.force(),2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png"); r.snap("agentview")
