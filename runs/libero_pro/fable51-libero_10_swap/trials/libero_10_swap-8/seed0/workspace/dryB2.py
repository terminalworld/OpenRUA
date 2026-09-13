import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.062
gam, sgn, pitch = float(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
spot = tuple(map(float, sys.argv[4:6])); phi = float(sys.argv[6]); stage = tuple(map(float, sys.argv[7:9]))
a = [-0.494, 0.87]
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.87, 0.494, 0.0])
t.grasp_R([0.235, -0.032, 0.967], a, math.radians(gam), sgn, math.radians(pitch))
t.lift(1.10)
t.place_auto([a[0], a[1], 0.0], spot, math.radians(phi))
print("DRY OK")
