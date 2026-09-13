import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
r.gripper(0.04)
q=quat_from_axes((0,0,-1),(1,0,0))
r.plan_go((0.037,0.030,1.20), q, vel=0.3, acc=0.3)
print("fingers",r.fingers())
rclpy.shutdown()
