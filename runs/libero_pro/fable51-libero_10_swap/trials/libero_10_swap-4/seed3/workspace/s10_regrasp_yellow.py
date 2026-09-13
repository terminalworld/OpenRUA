import numpy as np, math, sys
from rob import Robot, topdown_quat
r = Robot("s10")
MC = np.array([-0.011, 0.206, 0.55])   # mouth center estimate
R_WALL = 0.040
tcp_xy = [MC[0] + R_WALL, MC[1]]        # pinch the +x side of the rim
Z_GRASP = 0.54
yaw = None
for y_ in (math.pi/2, -math.pi/2):
    q = r.ik_world([tcp_xy[0], tcp_xy[1], Z_GRASP], topdown_quat(y_))
    print("yaw", y_, "IK", None if q is None else np.round(q, 3))
    if q is not None and yaw is None: yaw = y_
if yaw is None: raise SystemExit("no IK")
Q = topdown_quat(yaw)
def go(xyz, t):
    for attempt in range(3):
        q = r.ik_world(xyz, Q)
        if q is None: raise SystemExit(f"IK failed {xyz}")
        code, err = r.move_joints(q, t)
        if err < 0.02: return
        print("  retrying")
    raise SystemExit("no convergence")
r.gripper(0.04)
print("pre-grasp"); go([tcp_xy[0], tcp_xy[1], 0.70], 4.0)
print("tcp", np.round(r.tcp_pose_world()[0], 4), "hand q", np.round(r.tcp_pose_world()[1], 3))
if "--stop" in sys.argv: raise SystemExit(0)
print("descend to 0.60"); go([tcp_xy[0], tcp_xy[1], 0.60], 2.5)
base = r.read_wrench(); print("wrench", np.round(base, 3))
print("descend to grasp z"); go([tcp_xy[0], tcp_xy[1], Z_GRASP], 2.5)
w = r.read_wrench(); print("wrench", np.round(w, 3), "dFz", round(w[2]-base[2], 3))
print("tcp", np.round(r.tcp_pose_world()[0], 4))
