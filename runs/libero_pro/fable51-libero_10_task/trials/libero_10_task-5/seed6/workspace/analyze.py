import numpy as np, cv2
P=np.load("birdview_cloud.npy"); img=cv2.imread("birdview.png")
z=P[:,:,2]
print("table z median:", np.median(z[300:340, 250:400]))
# objects above table 0.89
mask=(z>0.895)&(P[:,:,0]>-0.6)&(P[:,:,0]<0.2)&(np.abs(P[:,:,1])<0.6)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i
    pts=P[m]
    print(i,"px area",stats[i,4],"bbox",stats[i,:4],"x[%.3f %.3f] y[%.3f %.3f] zmax %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
