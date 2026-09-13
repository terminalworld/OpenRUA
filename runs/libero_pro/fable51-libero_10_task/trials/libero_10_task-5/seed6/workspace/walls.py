import numpy as np
P=np.load("birdview_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
xs=np.arange(-0.49,-0.31,0.01); ys=np.arange(-0.37,0.08,0.01)
print("maxz (cm above table 0.880), rows=x, cols=y")
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.01)&(P[:,1]>=y)&(P[:,1]<y+0.01)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
