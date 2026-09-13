import numpy as np, sys
from ctl import *
r=Robot("rimcheck")
c,d,P,T=r.snap("robot0_eye_in_hand","/workspace/rc_eih.png")
np.save("rc_P.npy",P)
p,qq=r.hand_pose(); print("hand",p.round(4),qq.round(3))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
cx0,cy0=float(sys.argv[1]),float(sys.argv[2])
zlo=float(sys.argv[3]) if len(sys.argv)>3 else 0.975
sel=Q[(Q[:,2]>zlo)&(Q[:,2]<zlo+0.035)&(np.hypot(Q[:,0]-cx0,Q[:,1]-cy0)<0.075)]
print("rim pts",len(sel),"z range",sel[:,2].min().round(3),sel[:,2].max().round(3))
cx,cy=cx0,cy0
for it in range(6):
    A=np.c_[2*sel[:,0],2*sel[:,1],np.ones(len(sel))]; b=(sel[:,0]**2+sel[:,1]**2)
    s=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=s[0],s[1]; rad=np.sqrt(s[2]+cx**2+cy**2)
    res=np.abs(np.hypot(sel[:,0]-cx,sel[:,1]-cy)-rad); sel=sel[res<max(0.004,np.percentile(res,80))]
print("rim center %.4f %.4f r=%.4f n=%d"%(cx,cy,rad,len(sel)))
rr=np.hypot(sel[:,0]-cx,sel[:,1]-cy); print("radial pct 10/50/90/99",np.percentile(rr,[10,50,90,99]).round(4))
print("+x rim point x=%.4f (r_mid) ; hand x=%.4f ; offset hand-wall=%.4f"%(cx+rad,p[0],p[0]-(cx+rad)))
