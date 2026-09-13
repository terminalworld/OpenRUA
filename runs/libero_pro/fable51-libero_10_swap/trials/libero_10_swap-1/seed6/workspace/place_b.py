import numpy as np
from rob import *
r = Rob()
print("up"); r.move_tcp(-0.111, 0.041, 0.66, sec=3)
print("over basket"); r.move_tcp(0.016, 0.28, 0.66, sec=4)
print("fingers", r.fingers())
print("lower"); r.move_tcp(0.016, 0.28, 0.56, sec=3)
print("fingers", r.fingers())
r.gripper(0.04)
print("retreat"); r.move_tcp(0.016, 0.28, 0.70, sec=3)
r.shutdown()
