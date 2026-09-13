import numpy as np, rclpy, subprocess
from rob import Robot, quat_from_axes
r=Robot()
# pitched 45 deg hand pointing +y/down, fingers along x; camera sits 5cm along hand +x (= up/forward here)
q=quat_from_axes((0,0.707,-0.707),(1,0,0))
res=r.plan_go((-0.135,-0.50,1.05), q, vel=0.5, acc=0.5)
print("result", res)
rclpy.shutdown()
