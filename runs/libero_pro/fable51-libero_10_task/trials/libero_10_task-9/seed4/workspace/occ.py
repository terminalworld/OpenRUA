import numpy as np, sys
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
P=np.load(f"snaps/{cam}_world.npy").reshape(-1,3)
m=np.isfinite(P).all(1)&(P[:,2]>zlo)&(P[:,2]<zhi)
Q=P[m]
xs=np.arange(-0.45,0.30,0.02); ys=np.arange(-0.45,0.45,0.02)
print("occupancy (rows x from -0.45 up, cols y from -0.45 → +0.45), step 2cm; '#'=points")
print("      "+"".join(f"{y:+.2f}"[2] if i%5==0 else " " for i,y in enumerate(ys)))
print("      "+"".join(f"{abs(y)*100:03.0f}"[1] if i%5==0 else " " for i,y in enumerate(ys)))
for x in xs:
    row=""
    for y in ys:
        n=((Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)).sum()
        row+= "#" if n>=3 else ("." if n>0 else " ")
    print(f"{x:+.2f} {row}")
