import numpy as np, sys
from arm import Arm
arm = Arm("lifttest")
dz = float(sys.argv[1])
p0, q0 = arm.tcp_world()
arm.move_path([(p0 + np.array([0, 0, dz * i / 3]), q0) for i in range(1, 4)], secs=4)
print("fingers", arm.finger_gap())
