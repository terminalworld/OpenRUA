import numpy as np
P = np.load("robot0_eye_in_hand_xyz.npy"); Z=P[...,2]
xs = np.arange(-0.16,0.18,0.02); ys=np.arange(-0.06,0.30,0.02)
print("max z-0.90 in cm per 2cm cell; rows=x cols=y")
print("      "+" ".join(f"{int(round(y*100)):3d}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(P[...,0]>=x)&(P[...,0]<x+0.02)&(P[...,1]>=y)&(P[...,1]<y+0.02)&(Z<1.3)
        if m.sum()==0: row.append("  -"); continue
        z=Z[m].max()
        row.append("  ." if z<0.905 else f"{int(round((z-0.9)*100)):3d}")
    print(f"{x:5.2f} "+" ".join(row))
# fine measurement of drawer floor extents (z in [0.915,0.935])
f = P[(Z>0.915)&(Z<0.935)&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)]
print("floor x[%.3f,%.3f] y[%.3f,%.3f] z=%.4f"%(f[:,0].min(),f[:,0].max(),f[:,1].min(),f[:,1].max(),np.median(f[:,2])))
w = P[(Z>0.97)&(Z<0.99)&(P[...,0]>-0.15)&(P[...,0]<0.15)&(P[...,1]>0.0)&(P[...,1]<0.21)]
print("wall tops x[%.3f,%.3f] y[%.3f,%.3f] z=%.4f"%(w[:,0].min(),w[:,0].max(),w[:,1].min(),w[:,1].max(),np.median(w[:,2])))
# per-y profile along the drawer center x=0: what's the top z
print("profile x in [-0.01,0.01]:")
for y in np.arange(0.0,0.30,0.01):
    m=(P[...,0]>-0.01)&(P[...,0]<0.01)&(P[...,1]>=y)&(P[...,1]<y+0.01)&(Z<1.3)
    if m.sum(): print(f"  y={y:.2f} zmax={Z[m].max():.3f} zmin={Z[m].min():.3f} n={m.sum()}")
