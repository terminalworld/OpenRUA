import numpy as np, rob, sys
r = rob.Robot()
print("start q", np.round(r.arm_q(),3), "fingers", r.fingers(), flush=True)
p0, q0 = r.fk()
# 1. lift straight up above the bowl
print("== lift", flush=True)
if rob.move_cart(r, p0 + [0, 0, 0.14], q0, steps=4, seconds=3.0) is None: sys.exit("lift failed")
# 2. pre-pre-grasp: approach +x, fingers along y (hand y = -y), well behind the bottle
Q = rob.frame_quat([1, 0, 0], [0, -1, 0])
print("== to staging", flush=True)
q = r.move_pose(rob.hand_from_tcp([-0.32, 0.046, 1.15], Q), Q, seconds=6.0)
if q is None: sys.exit("staging IK failed")
rob.settle(r, q)
print("  TCP", rob.tcp_now(r).round(4), flush=True)
r.gripper(0.04)
# 3. pre-grasp
print("== pre-grasp", flush=True)
if rob.go_tcp(r, [-0.25, 0.046, 1.03], Q, steps=4, seconds=4.0) is None: sys.exit("pregrasp failed")
# 4. advance to the neck
print("== advance", flush=True)
if rob.go_tcp(r, [-0.150, 0.046, 1.03], Q, steps=4, seconds=4.0) is None: sys.exit("advance failed")
r.snap("robot0_eye_in_hand", "eih_pregrasp.png")
print("wrench before", r.wrench(), flush=True)
print("== close", flush=True)
f = r.gripper(0.0)
print("wrench after", r.wrench(), flush=True)
print("fingers", f, "q", np.round(r.arm_q(),3), flush=True)
print("DONE", flush=True)
