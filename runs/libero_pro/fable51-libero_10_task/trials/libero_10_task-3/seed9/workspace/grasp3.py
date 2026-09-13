import numpy as np
from rob import *
from goto import goto

r = Robot("g3")
R1 = np.load("snaps/R_side.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.04; az = np.radians(60)
r.gripper(0.04)
t, Rn = r.tcp(); print("start tcp", np.round(t, 3))
# back off along -Z of hand a bit and go to pre-grasp
pre = np.array([BOT[0] - 0.07 * np.cos(az), BOT[1] - 0.07 * np.sin(az), ZG])
goto(r, [pre[0] - 0.03 * np.cos(az), pre[1] - 0.03 * np.sin(az), ZG + 0.03], R1, seconds=4)
goto(r, pre, R1, seconds=3)
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre.png")
r.snap("agentview")
