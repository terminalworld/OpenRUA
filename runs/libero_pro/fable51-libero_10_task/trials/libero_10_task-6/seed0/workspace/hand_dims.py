import numpy as np
for cam in ["sideview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pc=d["pc"]; z=pc[...,2]
    # hand frame at tcp(-0.19,-0.05,0.72)+0.1034 up = z 0.823, yaw 90 -> fingers along x
    m=np.isfinite(z)&(np.abs(pc[...,0]+0.19)<0.2)&(np.abs(pc[...,1]+0.05)<0.15)&(z>0.72)&(z<0.95)
    P=pc[m]
    print(cam, "n", len(P))
    for zl in np.arange(0.72,0.95,0.01):
        s=(P[:,2]>=zl)&(P[:,2]<zl+0.01)
        if s.sum()>3: print(f"  z {zl:.2f}: n={s.sum():4d} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] w={P[s][:,0].max()-P[s][:,0].min():.3f}  y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}] w={P[s][:,1].max()-P[s][:,1].min():.3f}")
