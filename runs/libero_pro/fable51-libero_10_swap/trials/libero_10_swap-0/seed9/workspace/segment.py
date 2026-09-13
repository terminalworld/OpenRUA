import numpy as np, cv2, sys
cam=sys.argv[1]
P=np.load(f"{cam}_world.npy"); c=np.load(f"{cam}_rgb.npy")
z=P[...,2]
mask=(z>0.445)&(z<0.9)&np.isfinite(z)
# exclude robot: robot base around world x<-0.3 ... keep only x>-0.45? robot base at x=-0.51
mask&=(P[...,0]>-0.35)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=P[m]; col=c[m].mean(0)
    print(f"blob {i}: px area={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} meanBGR={col.astype(int)}")
cv2.imwrite(f"{cam}_mask.png",mask*255)
