import numpy as np
d=np.load("sideview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# points near the gripper fingers: around mug x, y, z 0.56-0.62
m=np.isfinite(z)&(np.abs(pc[...,0]+0.195)<0.12)&(np.abs(pc[...,1]-0.016)<0.06)&(z>0.545)&(z<0.60)
P=pc[m]; C=col[m]
print("n",m.sum())
# histogram along x at 5mm
xs=np.arange(-0.31,-0.08,0.005)
H,_=np.histogram(P[:,0],bins=xs)
for i,x in enumerate(xs[:-1]):
    sel=(P[:,0]>=x)&(P[:,0]<xs[i+1])
    if H[i]>0:
        print(f"x={x:+.3f} n={H[i]:4d} zmean={P[sel][:,2].mean():.3f} zmin={P[sel][:,2].min():.3f} BGR={C[sel].mean(0).round(0)}")
