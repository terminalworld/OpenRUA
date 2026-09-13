import rclpy, numpy as np
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0,-1),(1,0,0))
r.go((0.037,0.030,1.25), q, seconds=3.0, seed=r.arm_q())
print("hand", r.fk()[0], "fingers", r.fingers())
rclpy.shutdown()
