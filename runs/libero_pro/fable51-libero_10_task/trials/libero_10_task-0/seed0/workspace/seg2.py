import numpy as np, cv2
Wp=np.load('birdview_world.npy')
z=Wp[...,2]
mask=((z>0.44)&(z<0.75)&np.isfinite(z)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    pts=Wp[lab==i]
    print(i,'px area',stats[i,4],'centroid px',cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
