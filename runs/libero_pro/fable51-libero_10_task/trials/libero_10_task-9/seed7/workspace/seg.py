import numpy as np, cv2
d=np.load("snaps/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240; camz=3.0; camx=-0.2
h=camz-d  # world z
def w(u,v): Z=d[v,u]; return (camx+(v-cy)*Z/fx, (u-cx)*Z/fx, camz-Z)  # x=down in image, y=right
# objects above table (z>0.93) in region rows 150-420
mask=(h>0.93)&(h<1.5); mask[:150,:]=0; mask[:, :100]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    x,y,ww,hh,a=stats[i]
    if a<30: continue
    ys,xs=np.where(lab==i); zs=h[ys,xs]
    print(f"comp{i}: px bbox u[{x},{x+ww}] v[{y},{y+hh}] area={a} zmax={zs.max():.3f} zmed={np.median(zs):.3f} centroid_world={w(int(cent[i][0]),int(cent[i][1]))}")
    print("   corners world:", w(x,y)[:2], w(x+ww-1,y+hh-1)[:2])
