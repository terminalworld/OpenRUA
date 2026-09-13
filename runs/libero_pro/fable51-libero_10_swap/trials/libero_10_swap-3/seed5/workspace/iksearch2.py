from rlib import *
from scene import *
r = Robot("iks2")
sc = Scene(r.node)
rng = np.random.default_rng(1)
q_now = r.arm_q()
xc = -0.11
for tilt in [0, 10, 20, 30, 40]:
    th = np.deg2rad(tilt)
    Rp = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])  # tilt from vertical toward -y
    for ztip in [0.955, 0.975]:
        found = []
        for i in range(25):
            seed = q_now + rng.normal(0, 0.5, 7); seed[1] = 0.2; seed[6] = rng.uniform(-2.5, 2.5)
            try:
                q = r.ik([xc, -0.03, ztip], Rp, seed=seed)
            except Exception:
                continue
            if any(abs(q - s).max() < 0.05 for s in found):
                continue
            found.append(q)
        res = []
        for q in found:
            sc.publish(objects(drawer_front=-0.105, with_bowl=False))
            ok1, c1 = sc.check(ARM, q, 0.0)
            try:
                q2 = r.ik([xc, -0.172, ztip], Rp, seed=q)
            except Exception:
                res.append((q, ok1, c1, None, None, None)); continue
            sc.publish(objects(drawer_front=-0.23, with_bowl=False))
            ok2, c2 = sc.check(ARM, q2, 0.0)
            res.append((q, ok1, c1, q2, ok2, c2))
        print(f"== tilt {tilt} ztip {ztip}: {len(found)} sols")
        for q, ok1, c1, q2, ok2, c2 in res:
            print(f"   start j2={q[1]:.3f} ok={ok1} {c1[:2]}  q={np.round(q,3).tolist()}")
            if q2 is not None:
                print(f"     end j2={q2[1]:.3f} ok={ok2} {c2[:2]} q={np.round(q2,3).tolist()}")
