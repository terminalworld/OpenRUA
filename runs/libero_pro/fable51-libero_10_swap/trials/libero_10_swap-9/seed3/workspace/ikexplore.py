import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
print("H0",np.round(H0,3),"zh",np.round(zh,3))
r=Robot(); rng=np.random.default_rng(0)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def sols(p,q,n=40):
    out=[]
    for i in range(n):
        s=rng.uniform(lo,hi)
        try: sol=r.ik(p,q,seed=list(s),avoid=False,timeout=0.3)
        except RuntimeError: sol=None
        if sol: out.append(np.round(sol,2))
    return out
poses={"pinch+a":(H0,zh,a),"pinch-a":(H0,zh,-a),"pinchlift+a":(H0+[0,0,0.2],zh,a),"pinchlift-a":(H0+[0,0,0.2],zh,-a),
       "carry+Z":([-0.135,-0.402,1.03],[0,1,0],[0,0,1]),"carry-Z":([-0.135,-0.402,1.03],[0,1,0],[0,0,-1]),
       "pre+Z":([-0.135,-0.50,1.20],[0,1,0],[0,0,1]),"pre-Z":([-0.135,-0.50,1.20],[0,1,0],[0,0,-1]),
       "prelow+Z":([-0.135,-0.50,1.03],[0,1,0],[0,0,1]),"prelow-Z":([-0.135,-0.50,1.03],[0,1,0],[0,0,-1])}
for k,(p,z,y) in poses.items():
    q=quat_from_axes(np.array(z,float),np.array(y,float)); S=sols(np.array(p,float),q)
    print(k,len(S)); 
    for s in S[:6]: print("   ",s)
rclpy.shutdown()
