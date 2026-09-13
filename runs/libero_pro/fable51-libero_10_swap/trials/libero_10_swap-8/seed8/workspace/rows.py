import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
y0,y1=float(sys.argv[2]),float(sys.argv[3]); x0,x1=float(sys.argv[4]),float(sys.argv[5]); zt=float(sys.argv[6]) if len(sys.argv)>6 else 0.905
m=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,0]>x0)&(P[:,0]<x1)&(P[:,2]>zt)&(P[:,2]<1.1)
Q=P[m]
for x in np.arange(x0,x1,0.004):
    mm=(Q[:,0]>=x)&(Q[:,0]<x+0.004)
    if mm.sum()<3: continue
    R=Q[mm]; hi=R[R[:,2]>R[:,2].max()-0.01]
    print(f"x={x:+.3f} n={mm.sum():4d} y[{R[:,1].min():.3f},{R[:,1].max():.3f}] w={R[:,1].max()-R[:,1].min():.3f} zmax={R[:,2].max():.3f} ytop={hi[:,1].mean():.3f} ")
