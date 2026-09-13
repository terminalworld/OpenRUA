import numpy as np
from rob import *
from goto import goto

r = Robot("g4b")
R2 = np.load("snaps/R_g4.npy")
BOT = np.array([-0.166, 0.073]); ZG = 1.045
g = np.array([BOT[0], BOT[1], ZG])
goto(r, g, R2, seconds=4, via_n=2, resend=2, tol=0.005)
print("force", np.round(r.force(), 2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_g4.png")
r.snap("sideview")
r.gripper(0.0)
gap = r.finger_gap(); print("gap", round(gap, 4), "force", np.round(r.force(), 2))
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_g4c.png")
