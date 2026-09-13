import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0],[0.5,1.2,-0.5,-0.9,0.3,2.0,0.5]]
def test(p,q):
    for s in seeds:
        try:
            sol=r.ik(p,q,seed=s,avoid=False,timeout=0.5)
            if sol: return sol
        except RuntimeError: pass
    return None
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
def pinch(side,th,tin=0.012,zoff=0.022):
    s,ct=np.sin(np.radians(th)),np.cos(np.radians(th))
    z_h=-a*s-np.array([0,0,ct]); P=c+a*(0.053-tin)+side*perp*0.041+np.array([0,0,zoff]); H=P-0.093*z_h
    return tuple(H),quat_from_axes(z_h,perp)
for side in (-1,+1):
    for th in (30,35,40,45,50,60):
        H,q=pinch(side,th); sol=test(H,q)
        print(f"side{side:+d} th{th} H=({H[0]:+.3f},{H[1]:+.3f},{H[2]:.3f}) ", "OK" if sol is not None else "fail", np.round(sol,2) if sol is not None else "")
rclpy.shutdown()
