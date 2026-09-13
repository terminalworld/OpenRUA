import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
p,_=r.fk(); r.cart_path([((p[0],p[1],1.40),qv)],seconds_per_m=8)
tgt=(0.05,-0.10,1.35)
for t in range(3):
    r.plan_go(tgt,qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array(tgt))<0.01: break
print("hand",np.round(p,3)); rclpy.shutdown()
