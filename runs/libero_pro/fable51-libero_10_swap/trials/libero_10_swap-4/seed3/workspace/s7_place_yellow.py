import numpy as np
from rob import Robot, topdown_quat
r = Robot("s7")
Q = topdown_quat(0.0)
PLATE = np.array([-0.016, 0.305]); R_WALL = 0.039
xy = [PLATE[0], PLATE[1] - R_WALL]
def go(z, t):
    for attempt in range(3):
        q = r.ik_world([xy[0], xy[1], z], Q)
        if q is None: raise SystemExit(f"IK failed z={z}")
        code, err = r.move_joints(q, t)
        if err < 0.02: return
        print("  retrying")
    raise SystemExit("no convergence")
print("traverse at 0.80"); go(0.80, 4.5)
print("tcp", np.round(r.tcp_pose_world()[0], 4))
print("lower to 0.60"); go(0.60, 3.0)
base = r.read_wrench(); print("wrench", np.round(base, 3))
z = 0.60
while z > 0.505 + 1e-4:
    z = max(0.505, z - 0.01)
    go(z, 1.5)
    w = r.read_wrench(); tcp = r.tcp_pose_world()[0]
    print(f"z_cmd={z:.3f} tcp_z={tcp[2]:.4f} dFz={w[2]-base[2]:+.3f} fingers={np.round(r.fingers(),4)}")
    if abs(w[2] - base[2]) > 1.5:
        print("contact detected"); break
