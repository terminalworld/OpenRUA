import numpy as np
table=0.894
for cam in ["sideview","frontview","agentview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    print(cam)
    for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.14,-0.30,-0.12),"potB":(-0.12,0.0,0.14,0.32)}.items():
        m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
        print(" ",name)
        for lo in np.arange(0.0,0.17,0.01):
            mm=m&(Z>table+lo)&(Z<=table+lo+0.01)
            if mm.sum()<3: continue
            P=pw[mm]
            print(f"   z {lo:.2f}-{lo+0.01:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
