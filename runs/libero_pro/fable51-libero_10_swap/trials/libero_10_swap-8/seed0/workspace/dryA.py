import math, sys, numpy as np, recover
from recover import Recover
recover.GRASP_UP = 0.008
recover.Z_BOTTOM_TO_WAIST = 0.065
spot = tuple(map(float, sys.argv[1:3])); phi = float(sys.argv[3]); stage = tuple(map(float, sys.argv[4:6])); theta=float(sys.argv[6])
t = Recover(True); t.stage = stage; t.handle_world = np.array([0.136, -0.991, 0.0])
t.grasp([0.044, -0.058, 0.961], [-0.991, -0.136], 0.0, -1, math.radians(16))
t.lift(1.10)
t.place(math.radians(theta), spot, math.radians(phi))
print("DRY OK")
