from rlib import *
from scene import *
import itertools
r = Robot("iks3")
sc = Scene(r.node)
rng = np.random.default_rng(2)
q_now = r.arm_q()
sc.publish(objects(drawer_front=-0.23, with_bowl=False))   # end state (hardest)
for tilt, xc, ztip in itertools.product([0, 20, 40, 60, 80, 90], [-0.11, -0.15, -0.19], [0.96, 0.975]):
    th = np.deg2rad(tilt)
    Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
    found = []
    for i in range(20):
        seed = q_now + rng.normal(0, 0.5, 7); seed[0] = rng.uniform(-0.8, 0.8); seed[1] = rng.uniform(-0.5, 0.4); seed[6] = rng.uniform(-2.5, 2.5)
        try:
            q = r.ik([xc, -0.175, ztip], Rp, seed=seed, timeout=2)
        except Exception:
            continue
        if abs(q[0]) > 1.2 or any(abs(q - s).max() < 0.05 for s in found):
            continue
        found.append(q)
    for q in found:
        ok, c = sc.check(ARM, q, 0.0)
        print(f"tilt={tilt} x={xc} z={ztip} j2={q[1]:.3f} ok={ok} {c[:2]} q={np.round(q,3).tolist()}")
    if not found:
        print(f"tilt={tilt} x={xc} z={ztip}: none")
