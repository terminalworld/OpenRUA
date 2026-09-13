import numpy as np
d = np.load("agentview_cloud.npz"); Pw = d["pw"]; z = Pw[...,2]
def rep(name, m):
    pts = Pw[m]; print(f"{name}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f},{pts[:,2].mean():.4f})")
ok = np.isfinite(z)
rep("table near plate (y<-0.1, x 0.05..0.25)", ok & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(Pw[...,1]<-0.10)&(Pw[...,1]>-0.25)&(z<0.44))
rep("table near pudding (x -0.1..0, y 0.15..0.25)", ok & (Pw[...,0]>-0.1)&(Pw[...,0]<0.0)&(Pw[...,1]>0.15)&(Pw[...,1]<0.25)&(z<0.44))
rep("plate", ok & (Pw[...,0]>0.05)&(Pw[...,0]<0.25)&(np.abs(Pw[...,1])<0.12)&(z>0.425)&(z<0.5))
rep("pudding", ok & (Pw[...,0]>-0.10)&(Pw[...,0]<0.02)&(Pw[...,1]>0.02)&(Pw[...,1]<0.14)&(z>0.43)&(z<0.5))
rep("red mug", ok & (Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.43)&(z<0.6))
rep("white mug", ok & (Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>0.43)&(z<0.6))
# red mug rim points from agentview: highest band
m = ok & (Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.07)&(Pw[...,1]<0.11)&(z>0.555)
rep("red mug rim band", m)
