import numpy as np
d=np.load("birdview_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
m=(xyz[:,2]>0.888)&(xyz[:,2]<0.95)&(xyz[:,0]>-0.40)&(xyz[:,0]<-0.05)&(xyz[:,1]>0.0)&(xyz[:,1]<0.35)&(rgb.max(1)<70)
D=xyz[m]; print("dark n",len(D))
print("bbox x",D[:,0].min().round(3),D[:,0].max().round(3)," y",D[:,1].min().round(3),D[:,1].max().round(3)," z",D[:,2].min().round(3),D[:,2].max().round(3))
for lo in np.arange(D[:,2].min(),D[:,2].max(),0.005):
    k=(D[:,2]>=lo)&(D[:,2]<lo+0.005)
    if k.sum()<5: continue
    P=D[k]; print(f"z={lo:.3f} n={k.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
# corners via PCA of top face
top=D[D[:,2]>np.percentile(D[:,2],50)]
c=top[:,:2].mean(0); u,s,vt=np.linalg.svd(top[:,:2]-c,full_matrices=False)
proj=(top[:,:2]-c)@vt.T
print("top centre",c.round(3),"dirs",vt.round(3),"extent",(proj.max(0)-proj.min(0)).round(3))
