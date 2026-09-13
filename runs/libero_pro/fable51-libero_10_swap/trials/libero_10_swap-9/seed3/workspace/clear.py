import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
for tgt in [(0.05,-0.10,1.35),(0.0,0.0,1.35)]:
    if r.plan_go(tgt, qv, vel=0.3, acc=0.3): break
rclpy.shutdown()
