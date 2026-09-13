import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
X=-0.163; Z=1.048
for t in range(3):
    r.plan_go((X,-0.525,1.20),qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array([X,-0.525,1.20]))<0.01: break
r.cart_path([((X,-0.525,1.12),qv),((X,-0.525,Z),qv)],seconds_per_m=10)
y=-0.525
while y>-0.455:
    y+=0.012
    r.cart_path([((X,y,Z),qv)],seconds_per_m=30,min_step_t=1.2)
    p,_=r.fk(); lag=y-p[1]; print(f"target y {y:.3f} actual {p[1]:.3f} z {p[2]:.3f} lag {lag:.3f}")
    if lag>0.012 or p[2]>Z+0.012:
        print("stalled"); break
r.cart_path([((p[0],p[1],1.20),qv)],seconds_per_m=8)
rclpy.shutdown()
