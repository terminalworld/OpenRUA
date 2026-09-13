import rclpy, numpy as np
from arm import Arm, ARM
from fkj import fkj, a
sol = a.ik(-0.203, 0, 1.2696, 0.9239, 0.3827, 0, 0); print('ik:', sol)
if sol: fkj(sol)
