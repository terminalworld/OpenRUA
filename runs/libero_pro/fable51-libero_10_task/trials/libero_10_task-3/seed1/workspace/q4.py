import numpy as np
P = np.load("birdview_xyz.npy"); Z=P[...,2]
# grid summary: for region x in [-0.25,0.3], y in [-0.05,0.5], print max z per 2cm cell
xs = np.arange(-0.25,0.30,0.02); ys=np.arange(-0.05,0.50,0.02)
print("rows=x, cols=y (cm*100 of z above 0.90; '.' = table)")
print("      "+" ".join(f"{int(y*100):3d}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(P[...,0]>=x)&(P[...,0]<x+0.02)&(P[...,1]>=y)&(P[...,1]<y+0.02)
        if m.sum()==0: row.append("  -"); continue
        z=Z[m].max()
        row.append("  ." if z<0.905 else f"{int((z-0.9)*100):3d}")
    print(f"{x:5.2f} "+" ".join(row))
