import numpy as np, cv2
P=np.load("birdview_world.npy"); c=cv2.imread("birdview.png")
zw=P[...,2]
for (u,v) in [(200,200),(450,350),(250,330),(400,200),(330,330)]:
    print("table sample",u,v,P[v,u].round(3))
tz=np.median(zw[np.isfinite(zw)&(zw>0.40)&(zw<0.47)])
print("table median z",tz)
mask=(zw>tz+0.012)&np.isfinite(zw)&(zw<0.9)
num,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,num):
    if stats[i,4]<10: continue
    m=lab==i; pts=P[m]; col=c[m].mean(0)
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print(f"blob {i}: area {stats[i,4]} px({cents[i][0]:.0f},{cents[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} topmean ({top[:,0].mean():.3f},{top[:,1].mean():.3f}) bgr {col.astype(int)}")
