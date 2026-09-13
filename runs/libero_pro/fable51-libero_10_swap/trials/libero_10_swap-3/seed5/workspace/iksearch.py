from rlib import *
from scene import *
import sys
r = Robot("iks")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
th = np.deg2rad(float(sys.argv[1])) if len(sys.argv) > 1 else np.deg2rad(30)
Rpush = hand_R([1, 0, 0], [0, -np.cos(th), -np.sin(th)])
xc = -0.11
rng = np.random.default_rng(0)
q_now = r.arm_q()
sols = []
for i in range(60):
    seed = q_now + rng.normal(0, 0.6, 7)
    seed[6] = rng.uniform(-2.5, 2.5)
    try:
        q = r.ik([xc, 0.0, 0.955], Rpush, seed=seed)
    except Exception as e:
        continue
    if any(abs(q - s).max() < 0.05 for s in sols):
        continue
    sols.append(q)
    ok, c = sc.check(ARM, q, 0.0)
    # also end pose reachable from this q?
    try:
        q2 = r.ik([xc, -0.172, 0.955], Rpush, seed=q)
        ok2, c2 = sc.check(ARM, q2, 0.0)
        j2end = q2[1]
    except Exception:
        ok2, j2end = None, None
    print(f"j2={q[1]:.3f} valid={ok} {c[:1]} q={np.round(q,3).tolist()}  end j2={j2end} valid_end={ok2}")
