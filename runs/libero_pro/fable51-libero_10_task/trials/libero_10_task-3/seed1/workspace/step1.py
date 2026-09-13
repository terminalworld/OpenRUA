import numpy as np, rclpy
from ctl import *
c = Ctl()
c.gripper(GRIP["open_m"])
R_down = R_from_axes(zaxis=[0,0,-1], yaxis=[0,-1,0])   # same as current: hand X = world X
c.move([-0.05, 0.15, 1.36], R_down, secs=4.0)
print(fmt(*c.hand_pose()))
rclpy.shutdown()
