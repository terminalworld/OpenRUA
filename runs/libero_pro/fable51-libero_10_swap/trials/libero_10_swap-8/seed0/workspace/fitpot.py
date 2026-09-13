"""Fit lying pot B from birdview cloud: axis dir, extent, height profile."""
import numpy as np, sys
P=np.load("birdview_world.npy")
z=P[...,2]; ok=np.isfinite(z)
reg=ok&(P[...,0]>0.03)&(P[...,0]<0.35)&(P[...,1]>-0.12)&(P[...,1]<0.25)&(z>0.94)
pts=P[reg]
a=pts[(pts[:,2]>0.99)&(np.linalg.norm(pts[:,:2]-[0.156,-0.023],axis=1)<0.06)]
print("potA upper x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f -> center est (%.3f,%.3f)"%(a[:,0].min(),a[:,0].max(),a[:,1].min(),a[:,1].max(),a[:,2].max(),a[:,0].max()-0.0385,a[:,1].max()-0.0385))
b=pts[(np.linalg.norm(pts[:,:2]-[0.156,-0.023],axis=1)>0.06)&(np.linalg.norm(pts[:,:2]-[0.036,0.03],axis=1)>0.04)&(pts[:,1]>0.0)]
print("potB pts",len(b),"x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"%(b[:,0].min(),b[:,0].max(),b[:,1].min(),b[:,1].max(),b[:,2].min(),b[:,2].max()))
c=b[:,:2].mean(0); u,s,vt=np.linalg.svd(b[:,:2]-c,full_matrices=False)
d=vt[0]; proj=(b[:,:2]-c)@d
print("center",np.round(c,3),"axis dir",np.round(d,3),"extent",round(proj.min(),3),round(proj.max(),3))
for t in np.arange(proj.min(),proj.max(),0.01):
    s_=b[(proj>=t)&(proj<t+0.01)]
    if len(s_)==0: continue
    perp=(s_[:,:2]-c)@np.array([-d[1],d[0]])
    top=s_[np.argsort(s_[:,2])[-5:]]
    print(f"t={t:+.3f} n={len(s_):3d} zmax={s_[:,2].max():.3f} perp[{perp.min():+.3f},{perp.max():+.3f}] top-perp={np.mean((top[:,:2]-c)@np.array([-d[1],d[0]])):+.3f}")
