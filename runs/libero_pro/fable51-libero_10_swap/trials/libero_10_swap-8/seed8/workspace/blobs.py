import numpy as np, cv2
P=np.load("bird_world.npy"); zw=P[...,2]
img=cv2.imread("birdview.png")
table=zw[(zw>0.85)&(zw<0.9)]
print("table z median", np.median(table))
mask=((zw>0.905)&(zw<1.15)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]
    print(f"blob {i}: px area {stats[i,4]} centroid px {cent[i].round(1)} bbox {stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmed {np.median(pts[:,2]):.3f} color {img[m].mean(0).round()}")
