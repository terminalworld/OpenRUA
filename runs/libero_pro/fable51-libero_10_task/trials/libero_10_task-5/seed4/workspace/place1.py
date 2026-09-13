import numpy as np, sys
from arm import *
a = Arm("place1")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
print("now tcp", np.round(p0,4)); print("R0\n", np.round(R0,3))
# cup axis offset from TCP (measured): (+0.054, 0, -0.031)
OFF = np.array([0.054, 0.0, -0.031])
def tcp_for_axis(ax): return np.asarray(ax) - OFF
targets = {
 "via":  tcp_for_axis([-0.29, -0.11, 1.22]),
 "high": tcp_for_axis([-0.422, -0.176, 1.20]),
 "low":  tcp_for_axis([-0.422, -0.176, 1.105]),
}
qs = {}
seed = q0
for k, t in targets.items():
    q = a.ik(t, R0, seed=seed)
    if q is None: print(k, "IK FAIL"); sys.exit(1)
    p, R = a.tcp(q)
    print(k, "tcp", np.round(t,3), "fk", np.round(p,3), "Rerr", np.abs(R-R0).max().round(3), "jump", np.round(np.abs(q-seed).max(),3), "q", np.round(q,3))
    qs[k] = q; seed = q
np.save("place_qs.npy", np.array([qs["via"], qs["high"], qs["low"]]))
