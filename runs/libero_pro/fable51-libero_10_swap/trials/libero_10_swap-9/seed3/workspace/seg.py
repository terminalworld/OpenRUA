import numpy as np
d=np.load("birdview_depth.npy")
# camera at z=3.0 pointing straight down; world z = 3.0 - depth ; x = (v-cy)*Z/fy - 0.2 ; y=(u-cx)*Z/fx
import cv2
img=cv2.imread("birdview.png")
h,w=d.shape
fx=fy=None
import json
# read camera info intrinsics quickly from earlier: recompute from px mapping: use ratio
# instead just derive: from px(320,240)->(-0.2,0), px(400,400)->(0.38,0.29) at Z=2.1 : fy = 160*2.1/0.58
fy=160*2.1/0.58; fx=80*2.1/0.29
print("fx,fy",fx,fy)
def world(u,v):
    Z=d[v,u]; return ((v-240)*Z/fy-0.2,(u-320)*Z/fx,3.0-Z)
# microwave: things with world z between 1.0 and 1.2 in region u<300
zs=3.0-d
mask=(zs>1.0)&(zs<1.2)
ys,xs=np.where(mask)
print("microwave-height px range u:",xs.min(),xs.max()," v:",ys.min(),ys.max())
# print rows to show shape
for v in range(ys.min(),ys.max()+1,5):
    row=np.where(mask[v])[0]
    if len(row): print(v, row.min(), row.max(), len(row), "world x=%.3f"%world(row.min(),v)[0], "y range %.3f..%.3f"%(world(row.min(),v)[1],world(row.max(),v)[1]))
# mug: z between 0.93 and 1.0, u 300-360
m2=(zs>0.92)&(zs<1.0)
ys,xs=np.where(m2)
for lo,hi in [(290,370),(380,460)]:
    sel=(xs>=lo)&(xs<hi)
    if sel.any():
        uu=xs[sel];vv=ys[sel]
        print("mug px u",uu.min(),uu.max(),"v",vv.min(),vv.max(),"center world",world(int(uu.mean()),int(vv.mean())), "maxz",zs[vv,uu].max())
