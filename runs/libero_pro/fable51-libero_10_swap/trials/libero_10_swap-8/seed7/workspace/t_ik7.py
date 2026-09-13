from rob import *
from flip import R_of
r = Robot(); seed = r.arm_q()
H = math.pi/2
tests = [("hook start", (-0.16, 0.205, 1.016), 0.0, H), ("hook in", (-0.077, 0.205, 1.016), 0.0, H),
         ("hook arc mid", (-0.077, 0.16, 1.045), 0.0, H), ("hook arc end", (-0.077, 0.077, 1.048), 0.0, H),
         ("hook retreat", (-0.077, 0.077, 1.15), 0.0, H),
         ("waist pre", (-0.077, 0.26, 0.9615), -H, H), ("waist grasp", (-0.077, 0.14, 0.9615), -H, H),
         ("waist lift", (-0.077, 0.14, 1.20), -H, H), ("yaw mid", (-0.077, 0.14, 1.20), -H/2, H), ("yaw +x", (-0.077, 0.14, 1.20), 0.0, H),
         ("carry", (0.19, 0.10, 1.20), 0.0, H), ("place", (0.19, 0.10, 0.9955), 0.0, H), ("place2", (0.17, 0.10, 0.9955), 0.0, H),
         ("place tilt1.3", (0.19, 0.10, 0.9955), 0.0, 1.3)]
for name, tcp, yaw, tilt in tests:
    q = r.ik_tcp(tcp, R_of(yaw, tilt), seed=seed)
    print(f"{name}: {np.round(tcp,3)} -> {'NO' if q is None else np.round(q,2)}")
    if q is not None: seed = q
