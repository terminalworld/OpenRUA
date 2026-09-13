import numpy as np, sys
np.set_printoptions(linewidth=250, threshold=100000)
wz = np.load("snaps/bird_wz.npy"); table=0.899
for name,(u0,v0,w,h) in {"potA":(236,226,43,24),"potB":(361,265,43,24)}.items():
    sub = wz[v0:v0+h, u0:u0+w]-table
    print(name)
    for row in np.round(sub*100).astype(int): print(" ".join(f"{v:2d}" for v in row))
