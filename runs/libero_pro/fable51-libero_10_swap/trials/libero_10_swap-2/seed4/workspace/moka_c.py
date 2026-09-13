import sys, numpy as np, rclpy
from moka_plan import *
m = Mover("moka_c")
print("wrench before", np.round(wrench(m),2))
m.gripper(0.0)
print("fingers", m.fingers())
print("wrench after", np.round(wrench(m),2))
p,q = m.fk_world(); print("hand", np.round(p,4))
