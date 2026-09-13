import numpy as np
from scipy import ndimage
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
reg=np.isfinite(z)&(pc[...,0]>-0.3)&(pc[...,0]<0.0)&(np.abs(pc[...,1])<0.15)&(z>0.44)&(z<0.60)
lab,n=ndimage.label(reg)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<50: continue
    P=pc[m]; C=col[m]; vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px=({us.min()}-{us.max()},{vs.min()}-{vs.max()}) x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] BGR={C.mean(0).round(0)}")
    # body vs handle: histogram of y at mid height
    mid=m&(z>0.48)&(z<0.52)
    ys=pc[mid][:,1]; xs=pc[mid][:,0]
    print("   mid-height y range",ys.min().round(3),ys.max().round(3)," x range",xs.min().round(3),xs.max().round(3))
    # top rim
    top=m&(z>P[:,2].max()-0.01)
    print("   rim pts x",pc[top][:,0].min().round(3),pc[top][:,0].max().round(3)," y",pc[top][:,1].min().round(3),pc[top][:,1].max().round(3), " n",top.sum())
