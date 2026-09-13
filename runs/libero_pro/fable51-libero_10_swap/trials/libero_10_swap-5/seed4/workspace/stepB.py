from rob import *
r = Robot()
BOOK = np.array([-0.094, -0.019]); TOP = 1.075
TH = np.arctan2(0.779, 0.627) - np.pi
qd = topdown_quat(TH)
def goto(z, secs, tries=2):
    tgt = np.array([BOOK[0], BOOK[1], z])
    q, c = r.ik_world(tgt, qd, tcp=True)
    if q is None: raise SystemExit(f"IK fail {c}")
    for i in range(tries):
        code, err = r.move(q, secs)
        pos, quat, _, _ = r.fk_hand()
        tcp = pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
        print(f"  z={z}: code {code} jerr {err:.4f} tcp {np.round(tcp,4)}", flush=True)
        if err < 0.005: break
goto(1.12, 2.5); goto(1.035, 2.5)
print("close:", r.gripper(GRIP["closed_m"]), flush=True)
f1, f2 = r.fingers(); print("fingers after close", f1, f2, "gap", f1 - f2, flush=True)
goto(1.30, 3.0, tries=1)
print("fingers after lift", r.fingers(), flush=True)
