import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0.707,-0.707),(1,0,0))
for p in [(-0.10,-0.52,1.10),(-0.12,-0.50,1.06)]:
    res=r.plan_go(p, q, vel=0.3, acc=0.3)
    if res is not None: break
rclpy.shutdown()
