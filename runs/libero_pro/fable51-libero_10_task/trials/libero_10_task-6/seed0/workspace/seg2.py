import numpy as np, cv2
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
# table z near plate: pixels around plate but outside it
for (u,v) in [(334,430),(200,400),(450,400),(334,300),(100,250),(500,250)]:
    print("table px",(u,v),"->",pc[v,u].round(3))
# red mug: red-dominant pixels
b,g,r=col[...,0].astype(int),col[...,1].astype(int),col[...,2].astype(int)
red=(r>90)&(r>g+40)&(r>b+40)&np.isfinite(z)
lab,n=ndimage.label(red)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<100: continue
    P=pc[m]; vs,us=np.nonzero(m)
    print(f"red blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
# all points in the region of the red mug (x -0.35..0.0, y -0.1..0.1, z 0.43..0.65) excluding robot gray
reg=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.12)&(z>0.44)&(z<0.62)
P=pc[reg]; print("region n",reg.sum(), "x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
