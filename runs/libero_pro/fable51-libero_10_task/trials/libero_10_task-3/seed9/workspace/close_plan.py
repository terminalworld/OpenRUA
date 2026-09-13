import numpy as np, sys
from rob import *
from goto import ik_checked, margin
from scene import check

r = Robot("cp")
q0 = r.arm_q(); t0, R0 = r.tcp()
print("tcp", np.round(t0, 3), "gap", round(r.finger_gap(), 4))
res = {}
for tilt_deg in [45, 55, 65]:
    for xpush in [0.0, 0.06]:
        t = np.radians(tilt_deg)
        Z = np.array([0, np.cos(t), -np.sin(t)]); Y = np.array([1.0, 0, 0]); X = np.cross(Y, Z)
        R = np.column_stack([X, Y, Z])
        ok = True; qs = []
        for y in [0.02, 0.10, 0.19]:
            tcp = np.array([xpush, y, 0.957])
            q = ik_checked(r, tcp, R, qs[-1] if qs else q0, max_dist=3.0 if not qs else 0.8, tries=8)
            if q is None: print(f"tilt={tilt_deg} x={xpush} y={y}: no IK"); ok = False; break
            c = check(r, q)
            print(f"tilt={tilt_deg} x={xpush} y={y}: margin={margin(q):.2f} clear={c[0]:.3f} {c[1]}")
            qs.append(q)
            if c[0] < 0.0: ok = False
        if ok: res[(tilt_deg, xpush)] = (R, qs)
np.save("snaps/close_cands.npy", np.array([(k, v[0], v[1]) for k, v in res.items()], dtype=object), allow_pickle=True)
print("feasible:", list(res.keys()))
