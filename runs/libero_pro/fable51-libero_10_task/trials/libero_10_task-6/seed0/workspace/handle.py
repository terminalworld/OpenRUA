import numpy as np
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# handle region: -y side of the mug, beyond the body wall (y < 0.016-0.040-0.003)
m=np.isfinite(z)&(pc[...,0]>-0.28)&(pc[...,0]<-0.12)&(pc[...,1]<-0.028)&(pc[...,1]>-0.10)&(z>0.43)&(z<0.60)
P=pc[m]; print("handle pts",m.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
for zl in np.arange(0.43,0.60,0.01):
    s=(P[:,2]>=zl)&(P[:,2]<zl+0.01)
    if s.sum(): print(f"z {zl:.2f}: n={s.sum():3d} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}]")
