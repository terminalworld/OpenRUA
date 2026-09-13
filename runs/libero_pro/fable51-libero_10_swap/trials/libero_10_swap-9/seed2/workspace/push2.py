import numpy as np, rclpy
from rob import *
rclpy.init(); r=Robot()
print('tcp', r.tcp()[0], 'gap', r.finger_gap())
R=R_from_axes([0,0,-1],[0,1,0])
w0=wrench(r,3); print('w0',w0[:3])
goto(r,[-0.08,-0.40,1.071],R,seconds=3)
y=-0.40
while y<-0.336:
    y=round(y+0.005,3)
    ok=goto(r,[-0.08,y,1.071],R,seconds=1.5)
    w=wrench(r,3); dev=w[:3]-w0[:3]
    print(f'y={y:+.3f} tcp={np.round(r.tcp()[0],3)} dev={np.round(dev,2)}')
    if np.linalg.norm(dev)>6: print('contact force, stop'); break
print('final tcp', r.tcp()[0])
