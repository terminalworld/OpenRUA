from rob import *
r = Robot()
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
print("tcp", np.round(tcp_now(),4), "q", np.round(r.arm_q(),3), flush=True)
q, c = r.ik_world([-0.35, 0.30, 1.30], topdown_quat(-np.pi/2), tcp=True)
if q is None: raise SystemExit(f"IK fail {c}")
for t in range(2):
    code, err = r.move(q, 4.0)
    print("park: code", code, "jerr", round(err,4), "tcp", np.round(tcp_now(),4), flush=True)
    if err < 0.01: break
