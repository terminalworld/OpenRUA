import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q=np.array(q)  # quat x,y,z,w
R0=Rot.from_quat(q); M0=R0.as_matrix(); zh0=M0[:,2]; xh0=M0[:,0]
tip0=p+0.1034*zh0; print("hand",np.round(p,3),"tip",np.round(tip0,3),"xh",np.round(xh0,3),"zh",np.round(zh0,3),"fingers",r.fingers())
zt=np.array([0,1.0,0]); xt=np.array([0,0,1.0]) if xh0[2]>0 else np.array([0,0,-1.0]); yt=np.cross(zt,xt)
Mt=np.stack([xt,yt,zt],1); Rt=Rot.from_matrix(Mt); print("target xh",xt,"yh",yt)
tipt=np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv)>3 else np.array([-0.135,-0.42,1.09])
n=int(sys.argv[4]) if len(sys.argv)>4 else 6
sl=Slerp([0,1],Rot.concatenate([R0,Rt])); wps=[]
for i in range(1,n+1):
    s=i/n; Ri=sl([s])[0]; zi=Ri.as_matrix()[:,2]; tipi=tip0+(tipt-tip0)*s
    wps.append((list(tipi-0.1034*zi),list(Ri.as_quat())))
if len(sys.argv)>5 and sys.argv[5]=="go":
    qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=2.0,max_jump=0.6,avoid=False)
    print("ok" if qs is not None else "FAILED","fingers",r.fingers()); p,q=r.fk(); print("hand",np.round(p,3),"R",np.round(Rot.from_quat(q).as_matrix(),2))
else:
    seed=r.arm_q()
    for pos,quat in wps:
        try: qq=r.ik(pos,quat,seed=seed,timeout=3,avoid=False)
        except Exception: qq=None
        print(np.round(pos,3), None if qq is None else np.round(qq,2)); 
        if qq is not None: seed=qq
