#!/usr/bin/env python3
"""Move the wrist camera to (x, y, z) looking straight down, snap, and print a height map of
all non-table points in the view (cm above table) plus colour tags (Y yellow / W white / . other).
  python3 tools/look.py x y z [x0 x1 y0 y1] [cell]"""
import sys, subprocess, numpy as np, rclpy
sys.path.insert(0, "/workspace/tools")
import eih
from planner import Planner
from robot import mat_to_quat
from PIL import Image

x, y, z = map(float, sys.argv[1:4])
rclpy.init(); p = Planner()
p.set_scene(microwave="solid", yellow=False)
R = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
ok = p.goto_tcp(np.array([x, y, z]), mat_to_quat(R))
print("camera move ok", ok, "tcp", np.round(p.tcp()[0], 4))
subprocess.run("cd /workspace/snaps && python3 /workspace/tools/cam_snap.py robot0_eye_in_hand --depth >/dev/null", shell=True)
pos, quat, Rh = p.fk()
d = np.load("/workspace/snaps/robot0_eye_in_hand_depth.npy")
P = eih.cloud(d, pos, Rh).reshape(-1, 3)
img = np.array(Image.open("/workspace/snaps/robot0_eye_in_hand.png")).reshape(-1, 3).astype(int)
yellow = (img[:, 0] > 150) & (img[:, 1] > 110) & (img[:, 2] < 120)
white = (img[:, 0] > 150) & (img[:, 1] > 150) & (img[:, 2] > 150)
if len(sys.argv) > 7:
    x0, x1, y0, y1 = map(float, sys.argv[4:8])
else:
    x0, x1, y0, y1 = x - 0.15, x + 0.15, y - 0.15, y + 0.15
cell = float(sys.argv[8]) if len(sys.argv) > 8 else 0.01
m = (P[:, 0] >= x0) & (P[:, 0] < x1) & (P[:, 1] >= y0) & (P[:, 1] < y1) & (P[:, 2] > 0.905) & (P[:, 2] < 1.10)
nx, ny = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
H = np.full((nx, ny), -1.0); T = np.full((nx, ny), ".", dtype=object)
for (px, py, pz), yl, wh in zip(P[m], yellow[m], white[m]):
    i, j = int((px - x0) / cell), int((py - y0) / cell)
    if pz > H[i, j]:
        H[i, j] = pz; T[i, j] = "Y" if yl else ("W" if wh else "o")
print("rows x from %.3f, cols y from %.3f; height cm + tag" % (x0, y0))
print("       " + " ".join("%4d" % round((y0 + (j + 0.5) * cell) * 100) for j in range(ny)))
for i in range(nx):
    print("%6.3f " % (x0 + (i + 0.5) * cell) + " ".join(
        ("%3d%s" % (round((H[i, j] - 0.90) * 100), T[i, j])) if H[i, j] > 0 else "   ." for j in range(ny)))
Q = P[m & yellow]
if len(Q):
    print("yellow bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4), "n", len(Q))
Q = P[m & white]
if len(Q):
    print("white bbox", np.round(Q.min(0), 4), np.round(Q.max(0), 4), "n", len(Q))
p.destroy_node(); rclpy.shutdown()
