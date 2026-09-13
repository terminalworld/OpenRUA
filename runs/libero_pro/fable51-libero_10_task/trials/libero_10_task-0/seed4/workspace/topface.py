import sys, numpy as np, cv2
cam=sys.argv[1]; zmin=float(sys.argv[2]); zmax=float(sys.argv[3]); x0,x1,y0,y1=map(float,sys.argv[4:8])
depth=np.load(f"{cam}_depth.npy"); m=np.load(f"{cam}_meta.npz"); K,T=m["K"],m["T"]
H,W=depth.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-K[0,2])*depth/K[0,0],(vv-K[1,2])*depth/K[1,1],depth,np.ones_like(depth)],-1)@T.T
sel=(P[...,2]>zmin)&(P[...,2]<zmax)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
pts=P[sel]
print(f"n={sel.sum()} center=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
