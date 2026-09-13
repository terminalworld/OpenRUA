"""Carry the pendulum-hung yellow mug over the right plate and lower until base contact."""
import sys, math, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat
TCP_XY = [float(sys.argv[1]), float(sys.argv[2])] if len(sys.argv) > 2 else [0.0255, 0.285]
r = Robot("s13a"); Q = topdown_quat(math.pi / 2)
def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None: print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02: break
    return r.tcp_pose_world()[0]
def hold(t):
    r.move_joints(r.joints(), t)
tcp = r.tcp_pose_world()[0]
print("start", np.round(tcp, 4), "fingers", np.round(r.fingers(), 4))
if "--carry" in sys.argv:
    go([tcp[0], tcp[1], 0.80], 3.0)
    p = go([TCP_XY[0], TCP_XY[1], 0.80], 5.0); hold(3.0); print("over plate", np.round(p, 4))
    p = go([TCP_XY[0], TCP_XY[1], 0.60], 4.0); hold(2.0); print("at 0.60", np.round(p, 4), "fingers", np.round(r.fingers(), 4))
    base = r.read_wrench(); print("base wrench", np.round(base, 2))
    z = 0.60
    while z > 0.515:
        z -= 0.005
        p = go([TCP_XY[0], TCP_XY[1], z], 1.0)
        w = r.read_wrench(); d = w[2] - base[2]
        print(f"z={z:.3f} tcp={np.round(p,4)} dFz={d:+.2f} fingers={np.round(r.fingers(),4)}")
        if d > 0.8:
            print("contact"); break
