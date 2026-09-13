#!/usr/bin/env python3
"""Move the wrist camera above (x, y) and fit the lying mug (axis along x): prints rim x, axis y, handle bar centre."""
import sys, subprocess, numpy as np, rclpy
sys.path.insert(0, "/workspace/tools")
import eih
from planner import Planner
from robot import mat_to_quat
from PIL import Image
x, y = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); p = Planner()
p.set_scene(microwave="solid", yellow=False)
R = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
ok = p.goto_tcp(np.array([x, y, 1.33]), mat_to_quat(R))
subprocess.run("cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth >/dev/null", shell=True)
pos, quat, Rh = p.fk()
d = np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
P = eih.cloud(d, pos, Rh).reshape(-1, 3)
img = np.array(Image.open("/workspace/snaps/robot0_eye_in_hand.png")).reshape(-1, 3).astype(int)
yellow = (img[:, 0] > 150) & (img[:, 1] > 110) & (img[:, 2] < 120)
near = (P[:, 0] > x - 0.15) & (P[:, 0] < x + 0.15) & (P[:, 1] > y - 0.12) & (P[:, 1] < y + 0.12) & (P[:, 2] > 0.905) & (P[:, 2] < 1.10)
Q = P[yellow & near]
print("yellow n", len(Q), "bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4))
B = Q[Q[:, 2] < 0.995]
for xx in np.arange(B[:, 0].min(), B[:, 0].max(), 0.01):
    s = (B[:, 0] >= xx) & (B[:, 0] < xx + 0.01)
    if s.sum() > 5:
        lo, hi = np.percentile(B[s, 1], 3), np.percentile(B[s, 1], 97)
        print(f"x {xx:.3f} y {lo:.3f}..{hi:.3f} mid {(lo+hi)/2:.4f} w {hi-lo:.3f} zmax {B[s, 2].max():.3f}")
Hh = Q[Q[:, 2] > 1.02]
print("handle top n", len(Hh), "xy", np.round(Hh[:, :2].mean(0), 4), "zmax", round(Hh[:, 2].max(), 4))
p.destroy_node(); rclpy.shutdown()
