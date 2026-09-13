import sys, math, time, numpy as np
sys.path.insert(0, "/workspace")
from rob import Robot, topdown_quat

Z_GRASP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.535
r = Robot("s11")
Q = topdown_quat(math.pi / 2)
tcp = r.tcp_pose_world()[0]
xy = [tcp[0], tcp[1]]
print("start tcp", np.round(tcp, 4))

def go(xyz, t):
    for k in range(3):
        j = r.ik_world(xyz, Q, seed=r.joints())
        if j is None:
            print("IK fail", xyz); sys.exit(1)
        code, err = r.move_joints(j, t)
        if err < 0.02:
            break
    print("tcp", np.round(r.tcp_pose_world()[0], 4))

go([xy[0], xy[1], 0.60], 2.5)
base = r.read_wrench()
go([xy[0], xy[1], Z_GRASP], 2.5)
w = r.read_wrench()
print("dFz descend", round(w[2] - base[2], 2))
if abs(w[2] - base[2]) > 2.0:
    print("unexpected contact, abort"); sys.exit(2)
f = r.gripper(0.0)
print("fingers after close", f)
go([xy[0], xy[1], 0.62], 2.5)
print("fingers after lift1", r.fingers())
go([xy[0], xy[1], 0.80], 3.0)
print("fingers after lift2", r.fingers())
