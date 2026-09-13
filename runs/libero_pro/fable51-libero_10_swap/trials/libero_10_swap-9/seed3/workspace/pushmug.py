import numpy as np, rclpy, subprocess
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
r.gripper(0.04)
print("fingers", r.fingers())
# pre-pose above, then descend
for t in range(3):
    code=r.plan_go((-0.17,-0.527,1.20),qv,seed=None)
    p,_=r.fk(); print("plan_go code",code,"hand",np.round(p,3))
    if np.linalg.norm(p-np.array([-0.17,-0.527,1.20]))<0.01: break
qs=r.cart_path([((-0.17,-0.527,1.13),qv),((-0.17,-0.527,1.0584),qv)],seconds_per_m=10)
print("wrench before", np.round(r.wrench(),1) if hasattr(r,'wrench') else '')
# push +y in 2.5cm steps, checking
for y in (-0.502,-0.477):
    qs=r.cart_path([((-0.17,y,1.0584),qv)],seconds_per_m=20,min_step_t=1.5)
    print("fingers", r.fingers())
# lift
r.cart_path([((-0.17,-0.477,1.20),qv)],seconds_per_m=8)
p,_=r.fk(); print("final hand",np.round(p,3))
rclpy.shutdown()
