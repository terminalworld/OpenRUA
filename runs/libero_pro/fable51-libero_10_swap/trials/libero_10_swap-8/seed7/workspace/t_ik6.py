from rob import *
from flip import R_of
r = Robot(); seed = r.arm_q()
for tilt in (0.0, 0.2, 0.3, 0.45):
    for z in (1.25, 1.10):
        q = r.ik_tcp((0.05, 0.22, z), R_of(math.pi, tilt), seed=seed)
        print(f"yaw pi tilt {tilt} z {z}: {'NO' if q is None else np.round(q,2)}")
