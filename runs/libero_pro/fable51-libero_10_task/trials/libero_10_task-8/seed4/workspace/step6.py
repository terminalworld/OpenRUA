import numpy as np
from arm import Arm
a = Arm()
f = a.gripper(0.0)
for i in range(3):
    a.spin(0.2)
print("fingers settled", a.fingers())
