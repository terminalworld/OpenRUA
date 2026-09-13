import numpy as np
import rob
from rob import Robot
rob.BASE_IN_WORLD = np.zeros(3)   # test: assume MoveIt frame == world
r = Robot("s0b")
q0 = r.joints()
quat = (0.9996, 0.0, -0.0284, 0.0)
for label, pos in [("world-coords", (-0.053, 0.0, 0.7776)), ("base-coords", (0.457, 0.0, 0.3577))]:
    q = r.ik_world(pos, quat, at_tcp=False)
    print(label, None if q is None else np.round(q, 3), "current", np.round(q0, 3))
