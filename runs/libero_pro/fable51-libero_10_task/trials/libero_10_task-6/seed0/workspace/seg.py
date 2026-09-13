import numpy as np, sys, cv2
from scipy import ndimage
cam=sys.argv[1]
d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
valid=np.isfinite(z)
# table height: mode of z in the central region
zz=z[valid]; hist,edges=np.histogram(zz,bins=400,range=(0,1.5))
table_z=edges[np.argmax(hist)]
print("table z ~",round(table_z,4))
mask=valid&(z>table_z+0.008)&(z<table_z+0.4)
# exclude robot: x < -0.25 roughly (robot base at -0.51) -- keep but label
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<40: continue
    P=pc[m]; C=col[m]
    xmin,ymin,zmin=P.min(0); xmax,ymax,zmax=P.max(0)
    bgr=C.mean(0)
    vs,us=np.nonzero(m)
    print(f"blob{i}: n={m.sum()} px(u,v)=({us.mean():.0f},{vs.mean():.0f}) x[{xmin:.3f},{xmax:.3f}] y[{ymin:.3f},{ymax:.3f}] z[{zmin:.3f},{zmax:.3f}] center=({(xmin+xmax)/2:.3f},{(ymin+ymax)/2:.3f}) meanBGR={bgr.round(0)}")
