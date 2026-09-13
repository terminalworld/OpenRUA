import numpy as np
from kin import Robot
r = Robot("backoff")
p, qt, _ = r.tcp(); seed = r.arm_q(); r.report("start"); w0 = r.wrench()
for d in ([0.02, 0.01, 0.0], [0.03, 0.02, 0.03], [0.03, 0.02, 0.07], [0.03, 0.02, 0.10]):
    s = r.ik(p + d, qt, seed=seed); assert s
    seed = s; r.move(s, 1.5, retries=1); r.report(f"back {d}")
    print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
