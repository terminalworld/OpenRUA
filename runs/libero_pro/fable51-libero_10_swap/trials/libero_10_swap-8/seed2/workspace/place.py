"""Carry the grasped pot to a place spot and release.
Usage: python3 -u place.py <hx> <hy> <place_tcp_z> <yaw_from> <yaw_to>
(hx,hy) = target world xy of the handle bar (TCP); descends to place_tcp_z, opens, retreats."""
import sys
from rob import *

hx, hy, pz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw0, yaw1 = float(sys.argv[4]), float(sys.argv[5])
r = Robot("place")
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers", r.fingers(), flush=True)
start = pos - TCP * quat_R(*q_down_yaw(yaw0))[:, 2]  # current TCP
CARRY_Z = 1.20

def go(p, yaw, secs, seed):
    q = r.ik_world(p, q_down_yaw(yaw), seed=seed, at_tcp=True)
    if q is None:
        print("IK FAILED for", p, yaw, flush=True); sys.exit(2)
    code, err = r.move_joints(q, secs)
    pos, _ = r.fk_world()
    print(f"  -> {np.round(p,3)} yaw={yaw:.2f} code={code} err={err:.4f} tcp={np.round(pos - TCP*quat_R(*q_down_yaw(yaw))[:,2],4)} fingers={np.round(r.fingers(),4)}", flush=True)
    return q

seed = r.arm()
# 1. up to carry height
seed = go((start[0], start[1], CARRY_Z), yaw0, 3.0, seed)
# 2. interpolate xy + yaw in 4 steps at carry height
for k in range(1, 5):
    a = k / 4
    p = (start[0] + a * (hx - start[0]), start[1] + a * (hy - start[1]), CARRY_Z)
    seed = go(p, yaw0 + a * (yaw1 - yaw0), 3.0, seed)
# 3. descend
seed = go((hx, hy, pz + 0.06), yaw1, 3.0, seed)
seed = go((hx, hy, pz), yaw1, 3.0, seed)
print("open gripper ->", r.gripper(0.04), flush=True)
seed = go((hx, hy, pz + 0.10), yaw1, 3.0, seed)
print("DONE", flush=True)
