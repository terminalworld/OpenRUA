import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.062
spot = tuple(map(float, sys.argv[1:3])); phi = float(sys.argv[3]); stage = tuple(map(float, sys.argv[4:6]))
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.87, 0.494, 0.0])
t.grasp([0.235, -0.032, 0.967], [-0.494, 0.87], math.radians(-45), 1)
t.lift(1.10)
t.place(math.radians(180), spot, math.radians(phi))
print("DRY OK")
