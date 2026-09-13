import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]; D=d["D"]
Z=P[...,2]
zt=0.425
mask=((Z>zt+0.008)&(Z<zt+0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<10: continue
    m=lab==i
    pts=P[m]
    col=C[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px {cent[i].round(1)}, world cx {pts[:,0].mean():.3f} cy {pts[:,1].mean():.3f} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmin {pts[:,2].min():.3f} meanBGR {col.round()}")
