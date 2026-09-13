import sys, numpy as np
from arm import Arm
a = Arm("moveq")
q = np.array([float(v) for v in sys.argv[1].split(",")]); secs = float(sys.argv[2])
code = a.move_q(q, secs)
print("TCP", a.tcp_world()[0].round(4), "q", a.arm_q().round(3))
