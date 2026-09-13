import numpy as np
P = np.load("eih_cloud.npy"); d = np.load("eih1_depth.npy")
z = P[...,2]
# fingers: in lower band v>360, points much closer than table (z world > 1.1)
m = (z>1.15)
vs,us = np.where(m)
print("finger px u range", us.min(), us.max(), "v range", vs.min(), vs.max(), "n", m.sum())
s = P[m]
print("finger pts z range", s[:,2].min(), s[:,2].max())
left = s[s[:,0] < -0.2062]; right = s[s[:,0] > -0.2062]
print("left finger x range", left[:,0].min(), left[:,0].max(), "right finger x range", right[:,0].min(), right[:,0].max())
print("inner gap (right.min - left.max)", right[:,0].min()-left[:,0].max())
for lo in np.arange(1.15,1.36,0.02):
    mm = (s[:,2]>=lo)&(s[:,2]<lo+0.02)
    if mm.sum()>3:
        ss = s[mm]; l = ss[ss[:,0]<-0.2062]; rr = ss[ss[:,0]>-0.2062]
        if len(l) and len(rr): print(f"z[{lo:.2f}] n={mm.sum()} inner gap {rr[:,0].min()-l[:,0].max():.4f}  y range {ss[:,1].min():.3f},{ss[:,1].max():.3f}")
