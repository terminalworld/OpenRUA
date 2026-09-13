import numpy as np
pts=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.90)&(xyz[:,2]<1.0)&(xyz[:,0]>-0.30)&(xyz[:,0]<-0.09)&(xyz[:,1]>0.0)&(xyz[:,1]<0.135)&(rgb.min(1)>90)&~((rgb.max(1)<70))
    # exclude arm: arm is far (parked at y 0.35)
    pts.append(xyz[m])
B=np.concatenate(pts); print("body pts",len(B))
print("bbox x",B[:,0].min().round(3),B[:,0].max().round(3)," y",B[:,1].min().round(3),B[:,1].max().round(3)," z",B[:,2].min().round(3),B[:,2].max().round(3))
for lo in np.arange(B[:,0].min(),B[:,0].max(),0.01):
    m=(B[:,0]>=lo)&(B[:,0]<lo+0.01)
    if m.sum()<10: continue
    P=B[m]; k=P[:,2]>np.percentile(P[:,2],90)
    print(f"x={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ymid={(P[:,1].min()+P[:,1].max())/2:.3f} zmax={P[:,2].max():.3f} ridge_y={P[k,1].mean():.3f}")
