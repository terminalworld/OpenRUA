import numpy as np
wz = np.load("snaps/bird_wz.npy"); d = np.load("snaps/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
H,W = d.shape; vs,us = np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx
table=0.899
for name,(u0,v0,w,h) in {"potA":(236,226,43,24),"potB":(361,265,43,24),"knob":(320,292,25,26),"stove":(307,320,53,54)}.items():
    sub = wz[v0:v0+h, u0:u0+w]-table
    print(name); 
    print(np.round(sub*100).astype(int))
    for thr in [0.02,0.05,0.08,0.10,0.12,0.14]:
        m = np.zeros_like(wz,bool); m[v0:v0+h,u0:u0+w] = sub>thr
        if m.sum()==0: continue
        print(f"  >{thr}: n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] c=({wx[m].mean():.3f},{wy[m].mean():.3f})")
