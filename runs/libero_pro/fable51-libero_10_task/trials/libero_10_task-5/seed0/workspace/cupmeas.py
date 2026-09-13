import numpy as np, sys
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yellow=(rgb[:,0]-rgb[:,2])>50
white=(rgb.min(1)>120)&~yellow
print("white n",white.sum(),"yellow n",yellow.sum())
W=xyz[white]; Y=xyz[yellow]
print("white bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
print("yellow bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
# PCA of white body for axis
c=W.mean(0); u,s,vt=np.linalg.svd(W-c,full_matrices=False)
print("white centroid",c.round(3),"axis",vt[0].round(3),"sv",s.round(2))
# slices along the principal axis
a=vt[0]; t=(W-c)@a
for lo in np.arange(t.min(),t.max(),0.01):
    m=(t>=lo)&(t<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"t={lo:+.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
print("yellow centroid",Y.mean(0).round(3))
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
