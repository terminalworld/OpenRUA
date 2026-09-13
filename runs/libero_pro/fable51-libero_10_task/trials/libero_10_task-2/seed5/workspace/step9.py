import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
G = [0.012, -0.23]
r.move_pose([G[0], G[1], 0.98], Q, seconds=2.5, at_tcp=True)
r.move_pose([G[0], G[1], 0.91], Q, seconds=2.0, at_tcp=True)
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
