import numpy as np, cv2
W=np.load('birdview_world.npy'); z=W[...,2]
# table region: find z histogram between 0.5 and 1.2
zz=z[(z>0.5)&(z<1.2)]
h,e=np.histogram(zz,bins=140); i=np.argmax(h); table=(e[i]+e[i+1])/2; print('table z',table)
# objects above table
mask=(z>table+0.01)&(z<table+0.4)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for k in range(1,n):
    if stats[k,4]<30: continue
    m=lab==k
    pts=W[m]
    print(f'comp {k}: px={stats[k,4]} centroid px=({cent[k][0]:.0f},{cent[k][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f}')
