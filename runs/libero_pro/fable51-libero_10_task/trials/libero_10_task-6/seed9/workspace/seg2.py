import numpy as np, cv2, sys
cam=sys.argv[1]; table=float(sys.argv[2])
P=np.load(f"{cam}_xyz.npy"); c=np.load(f"{cam}_bgr.npy")
z=P[...,2]
obj=(z>table+0.012)&(z<table+0.35)&(P[...,0]>-0.4)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
obj=obj.astype(np.uint8)
nl,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for j in range(1,nl):
    if stats[j,cv2.CC_STAT_AREA]<15: continue
    mm=lab==j
    pts=P[mm]; col=c[mm].mean(0)
    print(f"blob{j} area={stats[j,4]} px_c=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round()}")
vis=c.copy(); vis[obj>0]=(0,255,0); cv2.imwrite(f"{cam}_seg.png",vis)
