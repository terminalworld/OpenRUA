import numpy as np
for cam in ["frontview","agentview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (Z>0.93)&(Z<1.13)&(X<-0.165)&(X>-0.6)&(Y>-0.2)&(Y<0.32)
    xs,ys,zs=X[m],Y[m],Z[m]
    # project onto door direction from hinge
    h = np.array([-0.17,0.28])
    dvec = np.stack([xs-h[0], ys-h[1]],1)
    ang = np.degrees(np.arctan2(dvec[:,1],dvec[:,0]))
    r = np.linalg.norm(dvec,axis=1)
    print(cam, "n", m.sum(), "angle pct", np.percentile(ang,[1,5,50,95,99]).round(1), "r max", r.max().round(3), "r pct", np.percentile(r,[50,90,99,99.9]).round(3))
    # tip: points with largest r
    idx = np.argsort(r)[-30:]
    print("  tip pts x,y,z:", np.c_[xs[idx],ys[idx],zs[idx]].mean(0).round(3))
    # thickness: perpendicular spread for a given angle
    a0 = np.median(ang); 
    perp = dvec @ np.array([-np.sin(np.radians(a0)), np.cos(np.radians(a0))])
    print("  median angle", a0.round(1), "perp spread pct", np.percentile(perp,[1,50,99]).round(3))
    print("  z range", zs.min().round(3), zs.max().round(3))
