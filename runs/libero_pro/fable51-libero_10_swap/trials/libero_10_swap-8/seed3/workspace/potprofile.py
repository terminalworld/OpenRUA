import numpy as np
pw=np.load("birdview_world.npy"); Z=pw[...,2]
table=0.894
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.14,-0.30,-0.12),"potB":(-0.12,0.0,0.14,0.32),"knob":(-0.03,0.1,-0.03,0.09),"stove":(0.08,0.30,-0.08,0.15)}.items():
    m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
    print(name)
    for lo in np.arange(0.01,0.18,0.01):
        mm=m&(Z>table+lo)
        if mm.sum()==0: continue
        P=pw[mm]
        print(f"  z>{lo:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] cx={P[:,0].mean():.3f} cy={P[:,1].mean():.3f}")
