import numpy as np, sys
from scene import grab
import rclpy
names=sys.argv[1:]
node,cams=grab(names)
res=0.005
x0,x1,y0,y1=-0.6,0.0,-0.45,0.25
W=int((x1-x0)/res); H=int((y1-y0)/res)
hm=np.full((H,W),np.nan)
for c in cams:
    P=c.cloud().reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]<1.2)
    P=P[sel]
    ix=((P[:,0]-x0)/res).astype(int); iy=((P[:,1]-y0)/res).astype(int)
    for a,b,z in zip(ix,iy,P[:,2]):
        if np.isnan(hm[b,a]) or z>hm[b,a]: hm[b,a]=z
np.save("hm.npy",hm)
# print coarse: rows=y (top=-0.45), cols=x
step=2
print("cols x from %.2f step %.3f"%(x0,res*step))
for j in range(0,H,step):
    row=hm[j,::step]
    s="".join(" ." if np.isnan(v) else ("%2d"%int(round((v-0.88)*100))) for v in row)
    print("y=%+.3f"%(y0+j*res), s)
rclpy.shutdown()
