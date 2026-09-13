"""Right the horizontally hanging pot B. Usage: python3 -u right2.py dry|place|release
Measured hang (TCP 0.235,0.096,1.198): base rim low point E at (0.124, 1.070): E-G = (-0.111,-0.128), |.|=0.169.
Upright with handle -x: G - E = (-0.123, 0.110). Arc about E from 49deg to 138deg, hand tilt fixed."""
import sys
from rob import *
Y = 0.096; T = 1.0; LIFT_Z = 1.20
E_X = 0.135                       # base edge on stove -> final pot centre ~0.170
R_ARC = 0.167
A0, A1 = math.atan2(0.128, 0.111), math.atan2(0.110, -0.123)
G0 = (E_X + 0.111, Y, 0.93 + 0.128)

def arc_points(n=10):
    return [((E_X + R_ARC*math.cos(A0 + (A1-A0)*k/n), Y, 0.93 + R_ARC*math.sin(A0 + (A1-A0)*k/n)), T - 0.4*k/n) for k in range(n+1)]

def go(r, tcp, tilt, seconds, seed=None, retry=True):
    q = r.ik_tcp(tcp, xbar_R(tilt), seed=seed if seed is not None else r.arm_q())
    if q is None: raise SystemExit(f"IK failed {np.round(tcp,3)} t{tilt}")
    print(f"-> tcp {np.round(tcp,3)} tilt {tilt:.2f} q {np.round(q,3)}", flush=True)
    code, err = r.move_q(q, seconds)
    if err > 0.02 and retry:
        print("  retry move", flush=True); code, err = r.move_q(q, seconds)
    pos, quat, t = r.fk_hand()
    print(f"  fk tcp now {np.round(t,4)} (err {np.linalg.norm(t-np.array(tcp))*1000:.1f} mm) fingers {np.round(r.fingers(),4)}", flush=True)
    return q

r = Robot()
mode = sys.argv[1]
print("fingers", r.fingers(), "G0", np.round(G0,4), "arc end", np.round(arc_points()[-1][0],4), flush=True)
if mode == "dry":
    seed = r.arm_q()
    for tcp, t in [((G0[0], Y, LIFT_Z), T), (G0, T)] + arc_points():
        q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
        print(np.round(tcp,3), f"t{t:.2f}", "NO" if q is None else f"{np.round(q,2)} jump {np.max(np.abs(np.array(q)-np.array(seed))):.2f}")
        if q is not None: seed = q
elif mode == "place":
    go(r, (G0[0], Y, LIFT_Z), T, 3.0)
    go(r, (G0[0], Y, G0[2] + 0.015), T, 3.0)
    go(r, G0, T, 2.0, retry=False)
    prev = r.arm_q()
    for tcp, t in arc_points()[1:]:
        prev = go(r, tcp, t, 1.5, seed=prev, retry=False)
    print("arc done; check pose before release", flush=True)
elif mode == "release":
    r.gripper(GRIP["open_m"])
    pos, quat, t = r.fk_hand()
    go(r, (t[0], t[1], t[2] + 0.06), T, 2.0)
    go(r, (t[0] - 0.05, t[1], LIFT_Z), 0.3, 3.0)
print("done", flush=True)
