"""Re-attempt grasp from the current hover position: open, descend, close, lift.
Usage: python3 -u regrasp.py <hx> <hy> <grasp_z> [yaw=0]"""
import sys
from rob import *

hx, hy, gz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
Q = q_down_yaw(yaw)
r = Robot("regrasp")
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("open gripper ->", r.gripper(0.04), flush=True)

q0 = r.arm()
q2 = r.ik_world((hx, hy, gz), Q, seed=q0, at_tcp=True)
print("grasp ik", None if q2 is None else np.round(q2, 3), flush=True)
print("descend ->", r.move_joints(q2, 4.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

print("close gripper ->", r.gripper(0.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

q3 = r.ik_world((hx, hy, gz + 0.08), Q, seed=q2, at_tcp=True)
print("lift ->", r.move_joints(q3, 3.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
