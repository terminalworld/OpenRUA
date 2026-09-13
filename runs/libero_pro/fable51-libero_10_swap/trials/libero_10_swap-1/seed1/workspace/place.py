#!/usr/bin/env python3
"""Carry the held object over the basket, measure the interior, release.
Usage: place.py bx by [release_z=0.65]"""
import sys
import subprocess
import numpy as np
from arm import *

bx, by = map(float, sys.argv[1:3])
rz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.65
Q = q_down_yaw(0.0)
a = Arm()
tp, _ = a.tcp()
print("start tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("transit over basket", flush=True)
mid = a.ik((tp[0], (tp[1] + by) / 2, 0.72), Q)
q_over = a.ik((bx, by, 0.75), Q, seed=mid)
a.move_q(q_over, 5.0, via=[(mid, 2.5)])
tp, _ = a.tcp(); print("  tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("measuring basket from eye-in-hand", flush=True)
subprocess.run([sys.executable, "scene.py", "robot0_eye_in_hand", "0.05"])
P = np.load("robot0_eye_in_hand_P.npy").reshape(-1, 3)
P = P[np.isfinite(P).all(1)]
rim = P[(P[:, 2] > 0.56) & (P[:, 2] < 0.66) & (abs(P[:, 0] - bx) < 0.15)
        & (abs(P[:, 1] - by) < 0.15)]
if len(rim):
    print(f"  rim pts n={len(rim)} x[{rim[:,0].min():.3f},{rim[:,0].max():.3f}] "
          f"y[{rim[:,1].min():.3f},{rim[:,1].max():.3f}] ztop={rim[:,2].max():.3f}",
          flush=True)
    cx, cy = (rim[:, 0].min() + rim[:, 0].max()) / 2, (rim[:, 1].min() + rim[:, 1].max()) / 2
    print(f"  rim centre ({cx:.3f},{cy:.3f})", flush=True)
else:
    cx, cy = bx, by
floor = P[(P[:, 2] < 0.5) & (abs(P[:, 0] - cx) < 0.05) & (abs(P[:, 1] - cy) < 0.05)]
if len(floor):
    print(f"  basket floor z ~ {np.percentile(floor[:,2],50):.3f} (n={len(floor)})", flush=True)

print("centre and lower", flush=True)
q_rel = a.ik((cx, cy, rz), Q, seed=a.arm_q())
a.move_q(q_rel, 3.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), "fingers", a.fingers(), flush=True)

print("release", flush=True)
a.gripper(0.04)
print("retreat up", flush=True)
q_up = a.ik((cx, cy, 0.75), Q, seed=a.arm_q())
a.move_q(q_up, 2.5)
print("  tcp", a.tcp()[0].round(4), flush=True)
print("DONE", flush=True)
