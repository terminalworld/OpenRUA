import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q=np.array(q); R0=Rot.from_quat(q); M0=R0.as_matrix(); zh0=M0[:,2]; xh0=M0[:,0]
tip0=p+0.1034*zh0
th=np.radians(float(sys.argv[1])); tipt=np.array([float(v) for v in sys.argv[2:5]]); n=int(sys.argv[5]); go=len(sys.argv)>6 and sys.argv[6]=="go"
zt=np.array([0,np.cos(th),-np.sin(th)]); xt=np.array([0,-np.sin(th),-np.cos(th)])
if xh0@xt<0: xt=-xt
yt=np.cross(zt,xt); Rt=Rot.from_matrix(np.stack([xt,yt,zt],1))
print("tip0",np.round(tip0,3),"->",tipt,"xh0",np.round(xh0,3),"xt",np.round(xt,3))
sl=Slerp([0,1],Rot.concatenate([R0,Rt])); wps=[]
for i in range(1,n+1):
    s=i/n; Ri=sl([s])[0]; zi=Ri.as_matrix()[:,2]; tipi=tip0+(tipt-tip0)*s
    wps.append((list(tipi-0.1034*zi),list(Ri.as_quat())))
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(5)
seed=r.arm_q(); allok=True; qs=[]
for pos,quat in wps:
    qq=None
    for k in range(8):
        sd=seed if k==0 else list(rng.uniform(lo,hi))
        try: qq=r.ik(pos,quat,seed=sd,timeout=0.5,avoid=False)
        except Exception: qq=None
        if qq is not None: break
    print(np.round(pos,3), None if qq is None else (np.round(qq,2), "jump",round(float(np.abs(np.array(qq)-np.array(seed)).max()),2)))
    if qq is None: allok=False
    else: seed=qq; qs.append(qq)
print("all reachable:",allok)
if go and allok:
    res=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.5,max_jump=0.8,avoid=False)
    print("exec", "ok" if res is not None else "FAILED", "fingers", r.fingers()); p,q=r.fk(); print("hand",np.round(p,3),"tip",np.round(p+0.1034*Rot.from_quat(q).as_matrix()[:,2],3))
