import numpy as np
P=np.load("bird_world.npy")
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.13,-0.30,-0.12),"potB":(-0.13,0.0,0.13,0.31),"stove":(0.06,0.30,-0.08,0.15),"knob":(-0.03,0.09,-0.03,0.09)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.90)
    pts=P[m]
    print(name, len(pts))
    for z in np.arange(0.90,1.07,0.01):
        s=pts[(pts[:,2]>=z)&(pts[:,2]<z+0.01)]
        if len(s)>3: print(f"  z {z:.2f}-{z+0.01:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
