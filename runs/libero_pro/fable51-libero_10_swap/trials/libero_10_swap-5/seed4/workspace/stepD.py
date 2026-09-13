from rob import *
r = Robot()
TH1 = -np.pi
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
def line(p0, p1, th, n, secs, tries=2):
    seed = r.arm_q(); wps = []
    for i in range(1, n+1):
        p = p0 + (p1-p0)*i/n
        q, c = r.ik_world(p, topdown_quat(th), seed=seed, tcp=True)
        if q is None: raise SystemExit(f"IK fail at {p}: {c}")
        wps.append(q); seed = q
    for t in range(tries):
        code, err = r.move(wps[-1], secs, via=wps[:-1] if t == 0 else None)
        print(f"  -> {np.round(p1,4)}: code {code} jerr {err:.4f} tcp {np.round(tcp_now(),4)} fingers {np.round(r.fingers(),4)}", flush=True)
        if err < 0.004: break
p0 = tcp_now(); print("tcp start", np.round(p0,4), flush=True)
line(p0, np.array([-0.433, -0.140, 1.075]), TH1, 3, 4.0)
tcp = tcp_now()
if abs(tcp[0] + 0.433) > 0.006 or abs(tcp[1] + 0.140) > 0.006 or abs(tcp[2] - 1.075) > 0.006:
    raise SystemExit(f"not aligned, aborting before release: {tcp}")
print("release:", r.gripper(GRIP["open_m"]), flush=True)
print("fingers", r.fingers(), flush=True)
line(tcp_now(), np.array([-0.433, -0.140, 1.30]), TH1, 2, 3.0, tries=1)
