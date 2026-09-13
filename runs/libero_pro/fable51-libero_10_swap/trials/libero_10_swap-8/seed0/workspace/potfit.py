"""Fit a lying pot's axis from a birdview cloud region. usage: potfit.py x0 x1 y0 y1 [zmin]"""
import sys, numpy as np
x0,x1,y0,y1=map(float,sys.argv[1:5]); zmin=float(sys.argv[5]) if len(sys.argv)>5 else 0.945
W=np.load("birdview_world.npy"); P=W.reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=P[(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)]
print("pts",len(A),"zmax",A[:,2].max().round(3))
c=A[:,:2].mean(0); u,s,vt=np.linalg.svd(A[:,:2]-c); a=vt[0]
for it in range(8):
    b=np.array([-a[1],a[0]]); w=(A[:,:2]-c)@b; t=(A[:,:2]-c)@a
    body=A[(np.abs(w)<0.037)&(np.abs(t)<0.09)]; tb=(body[:,:2]-c)@a
    keep=[]
    for t0 in np.arange(-0.09,0.09,0.005):
        s_=body[(tb>=t0)&(tb<t0+0.005)]
        if len(s_): keep.append(s_[s_[:,2]>s_[:,2].max()-0.006])
    ridge=np.vstack(keep); c=ridge[:,:2].mean(0); u,s,vt=np.linalg.svd(ridge[:,:2]-c); a=vt[0]
print("centre",np.round(c,3),"axis",np.round(a,3))
b=np.array([-a[1],a[0]]); t=(A[:,:2]-c)@a; w=(A[:,:2]-c)@b
for t0 in np.arange(-0.10,0.10,0.01):
    s_=(t>=t0)&(t<t0+0.01)&(np.abs(w)<0.06); S=A[s_]; ws=w[s_]
    if len(S):
        top=S[S[:,2]>S[:,2].max()-0.004]
        print(f"t {t0:+.2f}: n={len(S)} zmax={S[:,2].max():.3f} w(top)={((top[:,:2]-c)@b).mean():+.3f} w[{ws.min():+.3f},{ws.max():+.3f}]")
h=A[(np.abs(w)>0.037)&(np.abs(w)<0.10)]
if len(h): print("side pts (handle?)",len(h),"mean",np.round(h.mean(0),3),"w",np.round(((h[:,:2]-c)@b).mean(),3),"t range",np.round(((h[:,:2]-c)@a).min(),3),np.round(((h[:,:2]-c)@a).max(),3))
