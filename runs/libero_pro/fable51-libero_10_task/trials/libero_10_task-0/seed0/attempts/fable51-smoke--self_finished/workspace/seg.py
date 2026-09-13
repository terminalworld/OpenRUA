import numpy as np, cv2
Pw=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
z=Pw[...,2]
mask=((z>0.435)&(z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=Pw[m]
    col=img[m].mean(0)
    x0,y0,w,h=stats[i,:4]
    print(f"comp{i}: px({x0},{y0},{w}x{h}) area={m.sum()} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
