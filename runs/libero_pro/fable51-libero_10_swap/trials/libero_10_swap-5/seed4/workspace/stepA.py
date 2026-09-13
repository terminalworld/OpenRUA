from rob import *
r = Robot()
print("start q", np.round(r.arm_q(),3), "fingers", r.fingers(), flush=True)
print("open gripper:", r.gripper(GRIP["open_m"]), flush=True)
BOOK = np.array([-0.094, -0.019]); TOP = 1.075
TH = np.arctan2(0.779, 0.627) - np.pi       # closing axis along book thickness
qd = topdown_quat(TH)
pre = np.array([BOOK[0], BOOK[1], 1.20])
q, c = r.ik_world(pre, qd, tcp=True)
print("IK pre", c, None if q is None else np.round(q,3), flush=True)
if q is None: raise SystemExit("no IK")
code, err = r.move(q, 4.0)
print("move pre: code", code, "max joint err", round(err,4), flush=True)
pos, quat, _, _ = r.fk_hand()
print("hand now", np.round(pos,4), "tcp", np.round(pos + TCP*Rot.from_quat(quat).as_matrix()[:,2],4), flush=True)
