"""Close the bottom drawer: push its front panel face at x=-0.19 with the closed
fingertips (hand tilted 20 deg so the wrist leans +y), keeping joint2 <= 0.45."""
from rlib import *
r = Robot("s4c")
th = np.deg2rad(20)
Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xc, ztip, y0 = -0.19, 0.965, -0.04
r.report()
if r.finger() > 0.005:
    r.gripper(0.0)

def cart(waypoints, label):
    pts, frac = r.cartesian(waypoints)
    j2 = max(p[1] for p in pts)
    print(f"[{label}] fraction={frac:.3f} pts={len(pts)} max j2={j2:.3f}")
    if frac < 0.99 or j2 > 0.45:
        raise SystemExit(f"[{label}] path not acceptable")
    code, err = r.move_q(pts)
    q, p, R = r.report()
    t = p + TCP * R[:, 2]
    print(f"[{label}] tcp={np.round(t,4).tolist()} err={err:.4f} wrench={np.round(r.wrench(),2).tolist()}")
    return t, err

# 1. pre pose above, outside the drawer front
q_pre = r.ik([xc, y0, 1.10], Rp, seed=np.array([-0.34, 0.2, -0.09, -2.1, -0.88, 2.5, -0.38]), j2max=0.43)
print("q_pre", np.round(q_pre, 3).tolist())
r.move_q(q_pre); r.report()

# 2. descend to push height; must converge before any push
t, err = cart([([xc, y0, ztip], Rp)], "descend")
if abs(t[2] - ztip) > 0.005 or err > 0.02:
    raise SystemExit("descent did not converge; abort")
w0 = r.wrench()

# 3. push in segments, watch force
for yt in [-0.12, -0.16, -0.19, -0.215, -0.225]:
    t, err = cart([([xc, yt, ztip], Rp)], f"push->{yt}")
    f = r.wrench()[:3] - w0[:3]
    print(f"   dF={np.round(f,2).tolist()} |dF|={np.linalg.norm(f):.1f}")
    if np.linalg.norm(f) > 30 or (abs(t[1] - yt) > 0.01):
        print("   stopped: force/stall"); break

# 4. retreat: back +y a bit, then up
cart([([xc, t[1] + 0.03, ztip + 0.01], Rp), ([xc, y0, 1.10], Rp)], "retreat")
