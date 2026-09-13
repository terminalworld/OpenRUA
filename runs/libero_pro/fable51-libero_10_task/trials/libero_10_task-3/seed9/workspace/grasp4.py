import numpy as np
from rob import *
from goto import goto, ik_checked, margin

r = Robot("g4")
BOT = np.array([-0.166, 0.073]); ZG = 1.045
az = np.radians(135); tilt = np.radians(30)
d = np.array([np.cos(az), np.sin(az), 0.0])
Z = np.array([np.cos(tilt) * d[0], np.cos(tilt) * d[1], -np.sin(tilt)])
Y = np.array([-d[1], d[0], 0.0]); X = np.cross(Y, Z)
R2 = np.column_stack([X, Y, Z])
np.save("snaps/R_g4.npy", R2)
g = np.array([BOT[0], BOT[1], ZG])
pre = g - 0.07 * Z + np.array([0, 0, 0.03])
above = pre + np.array([0, 0, 0.09])

t0, R0 = r.tcp(); print("start tcp", np.round(t0, 3))
if r.finger_gap() < 0.07:
    r.gripper(0.04)
# 1. lift straight up with current orientation
goto(r, t0 + np.array([0, 0, 0.11]), R0, seconds=3, via_n=1, resend=1)
# 2. transit at height to above pre-grasp, rotating to R2
goto(r, above, R2, seconds=5, via_n=3, resend=1)
# 3. descend to pre
goto(r, pre, R2, seconds=3, via_n=1, resend=2)
r.snap("robot0_eye_in_hand", "/workspace/snaps/eih_pre4.png")
r.snap("agentview")
print("q", np.round(r.arm_q(), 3), "force", np.round(r.force(), 2))
