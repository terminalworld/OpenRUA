import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
tgt=tuple(float(v) for v in sys.argv[1:4]) if len(sys.argv)>3 else (0.05,-0.10,1.35)
for t in range(3):
    r.plan_go(tgt,qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array(tgt))<0.01: break
print("hand",np.round(p,3)); rclpy.shutdown()
