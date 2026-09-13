import numpy as np
for cam in ["agentview","birdview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]
    m=np.isfinite(z)&(pc[...,0]>-0.13)&(pc[...,0]<0.0)&(pc[...,1]>0.06)&(pc[...,1]<0.18)&(z>0.445)&(z<0.50)
    P=pc[m]; 
    if len(P)<10: print(cam,"few pts",len(P)); continue
    top=P[P[:,2]>P[:,2].max()-0.008]
    c=top[:,:2].mean(0); X=top[:,:2]-c
    w,v=np.linalg.eigh(X.T@X/len(X))
    ax=v[:,1]; ang=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=X@v
    print(f"{cam}: n={len(P)} top z={P[:,2].max():.3f} center=({c[0]:.3f},{c[1]:.3f}) long-axis angle={ang:.1f}deg  extents long={proj[:,1].max()-proj[:,1].min():.3f} short={proj[:,0].max()-proj[:,0].min():.3f}  zmin={P[:,2].min():.3f}")
