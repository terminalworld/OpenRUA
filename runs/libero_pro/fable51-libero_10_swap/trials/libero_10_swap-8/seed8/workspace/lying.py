import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([float(sys.argv[2]),float(sys.argv[3])]); zmin=float(sys.argv[4]) if len(sys.argv)>4 else 0.90
m=(np.linalg.norm(P[:,:2]-c,axis=1)<0.16)&(P[:,2]>zmin)&(P[:,2]<1.15)
Q=P[m]; print("n",len(Q),"zmax",Q[:,2].max().round(3))
# body axis from low points only (z<0.93) to avoid the handle
lo=Q[Q[:,2]<0.935]; cc=lo[:,:2].mean(0); U,S,Vt=np.linalg.svd(lo[:,:2]-cc,full_matrices=False); ax=Vt[0]; pe=Vt[1]
print("axis",ax.round(3),"az",np.degrees(np.arctan2(ax[1],ax[0])).round(1),"center_lo",cc.round(3))
a=(Q[:,:2]-cc)@ax; b=(Q[:,:2]-cc)@pe
for s in np.arange(a.min(),a.max(),0.01):
    mm=(a>=s)&(a<s+0.01)
    if mm.sum()<3: continue
    top=Q[mm][Q[mm,2].argmax()]
    print(f" a={s:+.3f} n={mm.sum():4d} zmax={Q[mm,2].max():.3f} at perp={b[mm][Q[mm,2].argmax()]:+.3f}  perp[{b[mm].min():+.3f},{b[mm].max():+.3f}]  perp(z<0.93)[{b[mm&(Q[:,2]<0.93)].min() if (mm&(Q[:,2]<0.93)).any() else np.nan:+.3f},{b[mm&(Q[:,2]<0.93)].max() if (mm&(Q[:,2]<0.93)).any() else np.nan:+.3f}]")
