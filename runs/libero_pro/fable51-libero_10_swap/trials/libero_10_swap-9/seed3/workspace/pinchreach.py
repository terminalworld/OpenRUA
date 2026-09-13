import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0]]
def test(p,q):
    for s in seeds:
        try:
            if r.ik(p,q,seed=s,avoid=False,timeout=0.5): return True
        except RuntimeError: pass
    return False
def pinch(c,a,side,th,tin=0.012,zoff=0.022):
    a=np.array([a[0],a[1],0.0]); a/=np.linalg.norm(a); perp=np.array([-a[1],a[0],0]); c=np.array(c,float)
    s,ct=np.sin(np.radians(th)),np.cos(np.radians(th))
    z_h=-a*s-np.array([0,0,ct]); P=c+a*(0.053-tin)+side*perp*0.041+np.array([0,0,zoff]); H=P-0.093*z_h
    return tuple(H),quat_from_axes(z_h,perp)
c=(-0.152,-0.42,0.95)
for name,a in [("current",(0.62,-0.785)),("rot85 (-x,-y)",(-0.7,-0.7)),("rot40 (-y)",(0,-1)),("rim->-x",(-1,0))]:
    for side in (-1,+1):
        row=[]
        for th in (30,45,60):
            H,q=test_args=pinch(c,a,side,th); ok=test(H,q); row.append(f"{th}:{'O' if ok else '.'}({H[0]:+.2f},{H[1]:+.2f})")
        print(f"{name:14s} side{side:+d} "+"  ".join(row))
rclpy.shutdown()
