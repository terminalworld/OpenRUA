import numpy as np
cx,cy=-0.0985,-0.2535
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X-cx)<0.08)&(np.abs(Y-cy)<0.08)&(Z>0.905)&(Z<1.03)&(Y<cy+0.02)
    xs,ys,zs=X[m],Y[m],Z[m]
    r=np.hypot(xs-cx,ys-cy)
    print(cam)
    for z in np.arange(0.90,1.03,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>3:
            rr=np.sort(r[mm])
            print(f"  z[{z:.2f}] n={mm.sum():4d} r_p50={rr[len(rr)//2]:.3f} r_p90={rr[int(len(rr)*0.9)]:.3f} r_max={rr[-1]:.3f}")
