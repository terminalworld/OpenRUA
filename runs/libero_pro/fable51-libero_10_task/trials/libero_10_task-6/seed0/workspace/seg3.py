import numpy as np
from scipy import ndimage
d=np.load("birdview_cloud.npz"); pc=d["pc"]; col=d["col"]; dep=d["dep"]
z=pc[...,2]
tbl=np.isfinite(z)&(np.abs(pc[...,0])<0.6)&(np.abs(pc[...,1])<0.8)&(z>0.3)&(z<0.46)
print("birdview table z median", np.median(z[tbl]).round(4), "n",tbl.sum())
mask=np.isfinite(z)&(np.abs(pc[...,0])<0.7)&(np.abs(pc[...,1])<0.9)&(z>0.44)&(z<1.5)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<5: continue
    P=pc[m]; C=col[m]; vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] ctr=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) BGR={C.mean(0).round(0)}")
# depth at plate center pixel
u,v=int(d["K"][0,2]+0.016*d["K"][0,0]/2.5),0
