import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from arm import Arm
a=Arm()
pos,q=a.fk()
print("current", np.round(pos,3), np.round(q,3))
print("ik current ->", a.ik(pos,q))
for p in [(0.4,0,0.4),(0.5,0,0.3),(0.3,0,0.5),(0.3,-0.13,0.4),(0.252,-0.134,0.4)]:
    print(p, "->", a.ik(p,[1,0,0,0]))
rclpy.shutdown()
