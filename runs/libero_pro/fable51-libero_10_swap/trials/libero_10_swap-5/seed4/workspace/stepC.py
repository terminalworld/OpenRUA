from rob import *
r = Robot()
TH0 = np.arctan2(0.779, 0.627) - np.pi       # current yaw of closing axis
TH1 = -np.pi                                  # closing axis along world x
PLACE = np.array([-0.433, -0.1375])
def tcp_now():
    pos, quat, _, _ = r.fk_hand(); return pos + TCP*Rot.from_quat(quat).as_matrix()[:,2]
def line(p0, p1, th0, th1, n, secs):
    """straight TCP line with yaw interpolation, one trajectory of n waypoints"""
    seed = r.arm_q(); wps = []
    for i in range(1, n+1):
        s = i/n; p = p0 + (p1-p0)*s; th = th0 + (th1-th0)*s
        q, c = r.ik_world(p, topdown_quat(th), seed=seed, tcp=True)
        if q is None: raise SystemExit(f"IK fail at {p} th={th}: {c}")
        if np.abs(q - seed).max() > 1.0: print("  WARN big joint jump", np.round(q-seed,2), flush=True)
        wps.append(q); seed = q
    code, err = r.move(wps[-1], secs, via=wps[:-1])
    print(f"  line -> {np.round(p1,4)}: code {code} jerr {err:.4f} tcp {np.round(tcp_now(),4)} fingers {np.round(r.fingers(),4)}", flush=True)
    return wps[-1]
p0 = tcp_now(); print("tcp start", np.round(p0,4), flush=True)
q = line(p0, np.array([PLACE[0], PLACE[1], 1.30]), TH0, TH1, 6, 6.0)
print("q above caddy", np.round(q,3), flush=True)
