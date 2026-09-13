import numpy as np
A = np.load("snaps/agentview_xyz.npy")
B = np.load("snaps/birdview_xyz.npy")
def ext(P, m, label):
    pts = P[m]
    if len(pts)==0: print(label, "none"); return
    print(f"{label}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    return pts
for name,P in [("agent",A),("bird",B)]:
    z=P[...,2]; x=P[...,0]; y=P[...,1]
    ext(P, (z>0.915)&(z<0.935)&(y>0)&(y<0.4)&(x>-0.3)&(x<0.3), name+" drawer floor")
    ext(P, (z>0.975)&(z<0.995)&(y>0)&(y<0.4)&(x>-0.3)&(x<0.3), name+" drawer rim")
    ext(P, (z>1.11)&(z<1.14)&(y>0)&(y<0.6)&(x>-0.3)&(x<0.3), name+" cabinet top")
    ext(P, (z>0.93)&(z<1.20)&(y>0.0)&(y<0.15)&(x>-0.25)&(x<-0.10), name+" bottle")
    ext(P, (z>1.10)&(z<1.20)&(y>0.0)&(y<0.15)&(x>-0.25)&(x<-0.10), name+" bottle top")
    ext(P, (z>0.93)&(z<1.20)&(y>-0.2)&(y<0.05)&(x>-0.1)&(x<0.15), name+" bowl")
