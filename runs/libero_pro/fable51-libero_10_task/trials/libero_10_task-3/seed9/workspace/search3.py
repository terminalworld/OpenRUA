import numpy as np
from rob import *
from goto import ik_checked, margin
from coll import clearance
r = Robot("s3")
q0 = r.arm_q()
c, w = clearance(r, q0); print("current clearance", round(c,3), w)
BOT = np.array([-0.166, 0.073]); ZG = 1.045
results = []
for az_deg in [45, 60, 75, 90, 105]:
    for tilt_deg in [0, 20, 35, 50]:
        az = np.radians(az_deg); tilt = np.radians(tilt_deg)
        d = np.array([np.cos(az), np.sin(az), 0.0])
        Z = np.array([np.cos(tilt)*d[0], np.cos(tilt)*d[1], -np.sin(tilt)])
        Y = np.array([-d[1], d[0], 0.0])         # closing axis horizontal
        X = np.cross(Y, Z)
        R = np.column_stack([X, Y, Z])
        g = np.array([BOT[0], BOT[1], ZG])
        pre = g - 0.07 * Z
        qg = ik_checked(r, g, R, q0, max_dist=3.0)
        if qg is None: print(az_deg, tilt_deg, "no IK grasp"); continue
        qp = ik_checked(r, pre, R, qg, max_dist=0.6)
        if qp is None: print(az_deg, tilt_deg, "no IK pre"); continue
        cg, wg = clearance(r, qg); cp, wp = clearance(r, qp)
        print(f"az={az_deg} tilt={tilt_deg} margin={margin(qg):.2f}/{margin(qp):.2f} clear={cg:.3f}/{cp:.3f} dq0={np.abs(qg-q0).max():.2f} worst={wg}")
        results.append((az_deg, tilt_deg, qg, qp, R))
np.save("snaps/search3.npy", np.array(results, dtype=object), allow_pickle=True)
