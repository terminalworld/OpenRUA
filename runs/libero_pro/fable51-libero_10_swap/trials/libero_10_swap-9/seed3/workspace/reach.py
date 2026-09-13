import numpy as np, rclpy, itertools
from rob import Robot, quat_from_axes
r=Robot()
cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8]]
def q_pitch(th): z=np.array([0,np.sin(np.radians(th)),-np.cos(np.radians(th))]); return quat_from_axes(z,(1,0,0))
for th in (30,45):
    q=q_pitch(th)
    for y in (-0.48,-0.50,-0.52,-0.54,-0.56):
        row=[]
        for z in (0.98,1.02,1.06,1.10):
            ok=False
            for s in seeds:
                try:
                    if r.ik((-0.14,y,z),q,seed=s,avoid=False,timeout=0.5): ok=True;break
                except RuntimeError: pass
            row.append('O' if ok else '.')
        print(f"pitch{th} y={y} z=.98/1.02/1.06/1.10 : {''.join(row)}")
rclpy.shutdown()
