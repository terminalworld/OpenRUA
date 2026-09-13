import numpy as np
P=np.load("bird_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
xs=np.arange(0.07,0.29,0.01); ys=np.arange(-0.08,0.14,0.01)
print("z(mm above 0.9) rows=y (top=+y), cols=x")
print("      "+" ".join(f"{x*100:3.0f}" for x in xs))
for y0 in ys[::-1]:
    row=[]
    for x0 in xs:
        m=(P[:,0]>=x0)&(P[:,0]<x0+0.01)&(P[:,1]>=y0)&(P[:,1]<y0+0.01)
        row.append(f"{(np.median(P[m,2])-0.9)*1000:3.0f}" if m.sum()>2 else "  .")
    print(f"{y0*100:4.0f}  "+" ".join(row))
