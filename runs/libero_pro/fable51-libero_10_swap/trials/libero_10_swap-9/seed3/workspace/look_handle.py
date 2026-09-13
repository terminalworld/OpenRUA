import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0.5,-0.866),(1,0,0))
res=r.plan_go((-0.01,-0.17,1.33), q, vel=0.3, acc=0.3)
rclpy.shutdown()
