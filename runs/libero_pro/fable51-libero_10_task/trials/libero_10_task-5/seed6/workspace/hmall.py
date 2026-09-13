import numpy as np, sys
cams=sys.argv[1:] or ["birdview"]
P=np.concatenate([np.load(f"g7_{c}_P.npy").reshape(-1,3) for c in cams]); P=P[np.isfinite(P).all(1)]
xs=np.arange(-0.72,-0.24,0.02); ys=np.arange(-0.42,0.16,0.02)
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.02)&(P[:,1]>=y)&(P[:,1]<y+0.02)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
