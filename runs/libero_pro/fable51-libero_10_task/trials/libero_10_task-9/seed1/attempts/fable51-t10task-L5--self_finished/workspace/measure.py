#!/usr/bin/env python3
"""Measure the white mug (rim circle, handle, height) and the door line from cached clouds."""
import numpy as np, sys

def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(-1,3)

def circle(xy):
    A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)

B=cloud('birdview'); A=cloud('agentview')
# white mug region (generous)
def mug(P, x0,x1,y0,y1, zmin=0.905):
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)
    return P[m]
for name,(x0,x1,y0,y1) in {'white':(-0.25,-0.02,-0.36,-0.12),'yellow':(-0.08,0.1,-0.12,0.08)}.items():
    b=mug(B,x0,x1,y0,y1); a=mug(A,x0,x1,y0,y1)
    if len(b)==0: print(name,'not found'); continue
    ztop=np.percentile(b[:,2],98)
    rim=b[b[:,2]>ztop-0.012]
    cx,cy,R=circle(rim[:,:2])
    res=np.hypot(rim[:,0]-cx,rim[:,1]-cy)-R; inl=np.abs(res)<0.006
    cx,cy,R=circle(rim[inl][:,:2])
    print(f'{name}: rim z={ztop:.3f} center=({cx:.4f},{cy:.4f}) R={R:.4f}  lowest(side cam)={a[:,2].min():.3f} n_bird={len(b)}')
    # handle: points beyond radius R+0.01 from center
    far=b[np.hypot(b[:,0]-cx,b[:,1]-cy)>R+0.008]
    if len(far):
        ang=np.degrees(np.arctan2(far[:,1]-cy,far[:,0]-cx))
        print(f'   handle: n={len(far)} x[{far[:,0].min():.3f},{far[:,0].max():.3f}] y[{far[:,1].min():.3f},{far[:,1].max():.3f}] z[{far[:,2].min():.3f},{far[:,2].max():.3f}] angle_med={np.median(ang):.0f}deg')
    fa=a[np.hypot(a[:,0]-cx,a[:,1]-cy)>R+0.008]
    if len(fa): print(f'   handle(agentview): y[{fa[:,1].min():.3f},{fa[:,1].max():.3f}] z[{fa[:,2].min():.3f},{fa[:,2].max():.3f}]')
# door: points at z in [0.95,1.09], x<-0.17, y in [-0.1,0.3]
d=B[(B[:,2]>0.95)&(B[:,2]<1.1)&(B[:,0]<-0.17)&(B[:,0]>-0.45)&(B[:,1]>-0.15)&(B[:,1]<0.3)]
if len(d):
    # fit line
    xy=d[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c); dirv=vt[0]
    t=(xy-c)@dirv
    p0=c+t.min()*dirv; p1=c+t.max()*dirv
    print(f'door: n={len(d)} ends ({p0[0]:.3f},{p0[1]:.3f}) -> ({p1[0]:.3f},{p1[1]:.3f}) ztop={d[:,2].max():.3f}')
