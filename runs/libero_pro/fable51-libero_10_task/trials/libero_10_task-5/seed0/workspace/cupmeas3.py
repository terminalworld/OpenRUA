import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.10)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yel=(rgb[:,0]>140)&(rgb[:,1]>110)&(rgb[:,2]<110)&((rgb[:,0]-rgb[:,2])>60)&(xyz[:,2]>0.90)
white=(rgb.min(1)>120)&~yel
W=xyz[white]; Y=xyz[yel]
print("white n",len(W),"bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
print("yellow n",len(Y),"bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
# white slices along x and along y
for lo in np.arange(W[:,0].min(),W[:,0].max(),0.01):
    m=(W[:,0]>=lo)&(W[:,0]<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"wx={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
for lo in np.arange(Y[:,0].min(),Y[:,0].max(),0.01):
    m=(Y[:,0]>=lo)&(Y[:,0]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yx={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
np.savez("cup_pts.npz", W=W, Y=Y)
