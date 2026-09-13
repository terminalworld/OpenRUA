"""Shift TCP in +y in small steps (mug base pinned by friction) to bring the mug upright, then release."""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
DY = float(sys.argv[1]); N = int(sys.argv[2]); DZ = float(sys.argv[3]) if len(sys.argv) > 3 else -0.003
r = Robot("s13b"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
tcp = r.tcp_pose_world()[0]; base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 2), "fingers", np.round(r.fingers(), 4))
for k in range(1, N + 1):
    p = go([tcp[0], tcp[1] + DY * k / N, tcp[2] + DZ * k / N], 1.5)
    w = r.read_wrench()
    print(f"step {k}: tcp={np.round(p,4)} dFz={w[2]-base[2]:+.2f} dFy={w[1]-base[1]:+.2f} fingers={np.round(r.fingers(),4)}")
    if w[2] - base[2] > 3.0:
        print("too much force, stop"); break
if "--release" in sys.argv:
    r.gripper(0.04)
    p = r.tcp_pose_world()[0]
    go([p[0], p[1], p[2] + 0.12], 3.0)
    print("released & lifted", np.round(r.tcp_pose_world()[0], 4))
