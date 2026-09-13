import sys, numpy as np
cam=sys.argv[1]; zmin,zmax,x0,x1,y0,y1=map(float,sys.argv[2:8])
depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
sel=(P[...,2]>zmin)&(P[...,2]<zmax)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts=P[sel][:,:2]; c=pts.mean(0); ev,evec=np.linalg.eigh(np.cov((pts-c).T))
long=evec[:,1]; ang=np.degrees(np.arctan2(long[1],long[0]))
proj=(pts-c)@evec
print(f"n={len(pts)} center=({c[0]:.4f},{c[1]:.4f}) long-axis angle from +x: {ang:.1f} deg; extents long={proj[:,1].max()-proj[:,1].min():.3f} short={proj[:,0].max()-proj[:,0].min():.3f} ztop={P[sel][:,2].max():.3f}")
