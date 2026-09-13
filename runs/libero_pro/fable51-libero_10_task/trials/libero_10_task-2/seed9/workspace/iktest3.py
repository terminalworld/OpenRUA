import numpy as np, rclpy
from arm import Arm
arm = Arm()
cur = np.array([-0.203,0,1.27]); q=(1,0,0,0)
for label, d in [("cur",(0,0,0)),("z-0.05",(0,0,-0.05)),("z-0.10",(0,0,-0.10)),("z-0.15",(0,0,-0.15)),("y+0.10",(0,0.10,0)),("y+0.19",(0,0.19,0)),("x+0.1",(0.1,0,0)),("x-0.1",(-0.1,0,0))]:
    try:
        sol = arm.solve_ik(cur+np.array(d), q); print(label,"OK",[round(v,3) for v in sol])
    except SystemExit as e: print(label,"->",e)
rclpy.shutdown()
