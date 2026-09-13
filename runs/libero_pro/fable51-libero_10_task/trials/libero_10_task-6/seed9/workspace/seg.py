import numpy as np, cv2
P=np.load("birdview_xyz.npy"); c=np.load("birdview_bgr.npy")
z=P[...,2]
# table top height: look at z values in the table region (0.3..0.6)
m=(z>0.3)&(z<0.6)
hist,edges=np.histogram(z[m],bins=60); i=hist.argmax(); table=(edges[i]+edges[i+1])/2
print("table z ~",table)
obj=(z>table+0.01)&(z<table+0.4)
obj=obj.astype(np.uint8)
nl,lab,stats,cent=cv2.connectedComponentsWithStats(obj)
for j in range(1,nl):
    if stats[j,cv2.CC_STAT_AREA]<15: continue
    mm=lab==j
    pts=P[mm]; col=c[mm].mean(0)
    print(f"blob{j} area={stats[j,4]} px_c=({cent[j][0]:.0f},{cent[j][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} bgr={col.round()}")
vis=c.copy(); vis[obj>0]=(0,255,0); cv2.imwrite("birdview_seg.png",vis)
