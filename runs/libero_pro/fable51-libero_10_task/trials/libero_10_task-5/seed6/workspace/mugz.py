import numpy as np, sys
from ctl import Robot
r=Robot("mugz")
for cam in ["agentview","sideview","frontview"]:
    c,d,P,T=r.snap(cam,f"/workspace/mugz_{cam}.png")
    pts=P.reshape(-1,3); col=c.reshape(-1,3).astype(int); ok=np.isfinite(pts).all(1); pts=pts[ok]; col=col[ok]
    m=(pts[:,0]>-0.63)&(pts[:,0]<-0.44)&(pts[:,1]>-0.05)&(pts[:,1]<0.2)&(pts[:,2]>0.886)&(pts[:,2]<1.06)
    b,g,rr=col[m].T
    gray=(np.abs(rr-g)<20)&(np.abs(g-b)<20)&(rr<170)
    q=pts[m][~gray]
    if len(q):
        h,e=np.histogram(q[:,2],bins=np.arange(0.88,1.07,0.01))
        print(cam,len(q),"zmin %.3f"%q[:,2].min(), [(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0][:6])
        lo=q[q[:,2]<q[:,2].min()+0.015]; print("   lowest band xy mean",lo[:,:2].mean(0).round(3), "x %.3f..%.3f y %.3f..%.3f"%(lo[:,0].min(),lo[:,0].max(),lo[:,1].min(),lo[:,1].max()))
    else: print(cam,"none")
