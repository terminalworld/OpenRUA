import numpy as np, rclpy
from handcheck import *
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
ELEV=26.0; ARM_T=0.020; RP=0.059
e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*np.array([0,0,1.0])
mug=(c,a,perp,rho)
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0],[0.5,1.2,-0.5,-0.9,0.3,2.0,0.5],[-0.5,1.2,0.5,-0.9,-0.3,2.0,-0.5],[0,1.3,0,-0.8,0,2.1,0.8],[1.0,0.3,-1.0,-2.0,0.3,2.2,-1.0],[-1.0,0.3,1.0,-2.0,-0.3,2.2,1.0]]
def ik(p,q):
    for s in seeds:
        try:
            sol=r.ik(p,q,seed=s,avoid=False,timeout=0.5)
            if sol: return sol
        except RuntimeError: pass
    return None
pad=c+ARM_T*a+RP*rho
print("pad centre",np.round(pad,3))
for ysign in (+1,-1):
    for tilt in (0,10,20):   # tilt of z_h about a toward -perp (away from handle side)
        th=np.radians(tilt); zh=-np.cos(th)*np.array([0,0,1.0])-np.sin(th)*perp*0  # placeholder
        zh=np.array([0,0,-1.0])*np.cos(th)+(-perp)*np.sin(th)
        Rm=pose_from(zh,ysign*a); H=pad-0.093*zh
        print(f"ysign{ysign:+d} tilt{tilt}: H={np.round(H,3)}")
        dep,what,Pw,lab=check(H,Rm,gap_half=0.04,mug=mug)   # open fingers
        cl=clearance(Pw,lab,c,a,perp); print("   clearances:",{k:round(v,3) for k,v in cl.items()})
        q=quat_from_axes(zh,ysign*a); sol=ik(H,q); print("   IK:", "OK "+str(np.round(sol,2)) if sol is not None else "fail")
        # lifted pose
        sol2=ik(H+np.array([0,0,0.20]),q); print("   IK lifted:", "OK" if sol2 is not None else "fail")
# carry poses
for name,H,zh,yh in [("carry",(-0.135,-0.402,1.03),(0,1,0),(0,0,1)),("precarry",(-0.135,-0.55,1.25),(0,1,0),(0,0,1)),("precarry_low",(-0.135,-0.55,1.03),(0,1,0),(0,0,1)),
                     ("rot90",(-0.20,-0.35,1.25),tuple(a),(0,0,1)),("carry_yneg",(-0.135,-0.402,1.03),(0,1,0),(0,0,-1))]:
    q=quat_from_axes(np.array(zh,float),np.array(yh,float)); sol=ik(np.array(H),q); print(f"{name}: IK", "OK "+str(np.round(sol,2)) if sol is not None else "fail")
rclpy.shutdown()
