import math, sys, numpy as np
from knob import Knob
k = list(map(float, sys.argv[1:9])); spot = (float(sys.argv[9]), float(sys.argv[10])); drop = float(sys.argv[11])
t = Knob(True)
t.pick(k[0:3], k[3:5], math.radians(k[5]), int(k[6]), math.radians(k[7]))
t.place(spot, drop)
print("DRY OK")
