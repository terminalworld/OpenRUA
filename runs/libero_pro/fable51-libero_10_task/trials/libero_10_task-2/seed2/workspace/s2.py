from rob import *
import subprocess
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
r.grip(0.04)
print("goto knob_pre", r.goto(-0.197, 0.201, 1.05, Q_DOWN_Y, 4.0))
print("joints", np.round(r.joints(),3))
