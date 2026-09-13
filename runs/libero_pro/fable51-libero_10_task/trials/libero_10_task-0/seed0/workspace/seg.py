import numpy as np, cv2
Wp=np.load('robot0_robotview_world.npy')
C=cv2.imread('robot0_robotview.png')
z=Wp[...,2]
# table height: mode of z
hist,edges=np.histogram(z[np.isfinite(z)],bins=200)
tz=edges[np.argmax(hist)]; print('table z ~',tz)
mask=(z>tz+0.01)&np.isfinite(z)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<150: continue
    m=lab==i
    pts=Wp[m]
    print(i,'px area',stats[i,4],'centroid px',cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3),'mean',pts.mean(0).round(3))
