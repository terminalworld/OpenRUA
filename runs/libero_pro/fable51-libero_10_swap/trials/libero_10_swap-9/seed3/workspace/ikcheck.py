import numpy as np, rclpy
from rob import Robot, quat_from_axes
from ins_geom import pose
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
q33,info=pose(33,-0.17)
H=info['H']; H0=H+np.array([0,-0.10,0])
tests=[("lift",(0.037,0.03,1.36),qv),("over",(-0.119,-0.48,1.36),qv),("tilted high",(-0.119,-0.48,1.36),q33),
       ("H0",H0,q33),("H",H,q33)]
for th,y in ((45,-0.426),(60,-0.441),(60,-0.391),(45,-0.386)):
    z_h=np.array([0,np.sin(np.radians(th)),-np.cos(np.radians(th))])
    Hd=np.array([-0.17,y,1.0])-0.093*z_h
    tests.append((f"drag{th} P_y={y}",Hd,quat_from_axes(z_h,(1,0,0))))
seed=r.arm_q()
for name,p,q in tests:
    s=r.ik(p,q,seed=seed,avoid=True)
    s2=r.ik(p,q,seed=seed,avoid=False) if s is None else s
    print(f"{name:20s} {np.round(p,3)} avoid={'OK' if s else 'FAIL'} free={'OK' if s2 else 'FAIL'}")
    if s: seed=s
rclpy.shutdown()
