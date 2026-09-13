"""Pick a moka pot by its handle bar (top-down grasp) and lift.
Usage: python3 -u pick.py <hx> <hy> [grasp_z=1.00]
hx,hy = world xy of the handle's outer bar; hand yaw 0 (fingers close along x)."""
import sys
from rob import *

hx, hy = float(sys.argv[1]), float(sys.argv[2])
gz = float(sys.argv[3]) if len(sys.argv) > 3 else 1.00
Q = q_down_yaw(0.0)
r = Robot("pick")
print("start q", np.round(r.arm(), 3), flush=True)

print("open gripper ->", r.gripper(0.04), flush=True)

hover = (hx, hy, 1.15)
q1 = r.ik_world(hover, Q, at_tcp=True)
print("hover ik", None if q1 is None else np.round(q1, 3), flush=True)
print("move hover ->", r.move_joints(q1, 4.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

q2 = r.ik_world((hx, hy, gz), Q, seed=q1, at_tcp=True)
print("grasp ik", None if q2 is None else np.round(q2, 3), flush=True)
print("descend ->", r.move_joints(q2, 3.0), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)

print("close gripper ->", r.gripper(0.0), flush=True)
print("fingers", r.fingers(), flush=True)

q3 = r.ik_world((hx, hy, gz + 0.08), Q, seed=q2, at_tcp=True)
print("lift ->", r.move_joints(q3, 2.5), flush=True)
pos, _ = r.fk_world(); print("hand now", np.round(pos, 4), "tcp z", round(pos[2] - TCP, 4), flush=True)
print("fingers after lift", r.fingers(), flush=True)
print("DONE", flush=True)
