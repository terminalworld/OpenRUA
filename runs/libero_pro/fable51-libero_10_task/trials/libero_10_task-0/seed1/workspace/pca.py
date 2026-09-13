import numpy as np
for cam, box in [("birdview", None), ("agentview", None)]:
    pw = np.load(f"{cam}_xyz.npy")
    # cream cheese
    for name, (x0,x1,y0,y1,zmin) in {"cheese": (0.03,0.15,-0.23,-0.15,0.432), "can": (-0.13,-0.03,0.0,0.09,0.43), "butter": (0.02,0.12,0.0,0.07,0.43), "basket": (-0.1,0.12,0.15,0.36,0.5)}.items():
        m = (pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&(pw[...,2]>zmin)&(pw[...,2]<0.8)
        pts = pw[m]
        if len(pts) < 10: print(cam, name, "few pts", len(pts)); continue
        xy = pts[:,:2]; c = xy.mean(0)
        top = pts[pts[:,2] > pts[:,2].max()-0.012]
        ev, evec = np.linalg.eigh(np.cov((top[:,:2]-top[:,:2].mean(0)).T))
        ang = np.degrees(np.arctan2(evec[1,1], evec[0,1]))
        print(f"{cam} {name}: n={len(pts)} mean xy=({c[0]:.3f},{c[1]:.3f}) top n={len(top)} top mean=({top[:,0].mean():.3f},{top[:,1].mean():.3f}) ztop={pts[:,2].max():.3f} top extents x[{top[:,0].min():.3f},{top[:,0].max():.3f}] y[{top[:,1].min():.3f},{top[:,1].max():.3f}] major axis angle {ang:.1f} deg, sd {np.sqrt(ev)}")
