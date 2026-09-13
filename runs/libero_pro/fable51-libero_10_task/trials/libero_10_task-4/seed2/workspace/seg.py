import numpy as np, cv2, sys
from percep import cloud
from scipy import ndimage
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
color,depth,P=cloud(cam)
z=P[...,2]
mask=(z>0.44)&(z<0.75)&(P[...,0]>-0.35)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<15: continue
    pts=P[m]; c=color[m].mean(0)[::-1]
    vs,us=np.where(m)
    print(f"blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} rgb={c.astype(int)} xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f})")
vis=color.copy(); vis[mask]=(0,255,0)
cv2.imwrite(f"snaps/{cam}_seg.png",vis)
