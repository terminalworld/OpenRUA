import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
r = Robot("s13c"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
tcp = r.tcp_pose_world()[0]; base = r.read_wrench()
print("start", np.round(tcp, 4), "wrench", np.round(base, 2))
z = tcp[2]
while z > 0.53:
    z -= 0.002
    p = go([tcp[0], tcp[1], z], 1.0); w = r.read_wrench()
    print(f"z={z:.4f} tcp={np.round(p,4)} dFz={w[2]-base[2]:+.2f}")
    if w[2] - base[2] > 0.3: print("light contact"); break
r.gripper(0.04)
p = r.tcp_pose_world()[0]
go([p[0], p[1], p[2] + 0.15], 3.0)
print("released & lifted", np.round(r.tcp_pose_world()[0], 4))
