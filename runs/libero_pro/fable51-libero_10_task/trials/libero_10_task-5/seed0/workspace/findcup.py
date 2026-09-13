import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.3)&(xyz[:,1]>-0.5)&(xyz[:,1]<0.5)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
yel=(rgb[:,0]>140)&(rgb[:,1]>110)&(rgb[:,2]<110)&((rgb[:,0]-rgb[:,2])>60)&(xyz[:,2]>0.90)
Y=xyz[yel]; print("yellow n",len(Y))
if len(Y): print(" bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3), "centroid", Y.mean(0).round(3))
# white near yellow centroid
c=Y.mean(0)
near=(np.linalg.norm(xyz[:,:2]-c[:2],axis=1)<0.12)
white=(rgb.min(1)>120)&~yel&near
W=xyz[white]; print("white n",len(W)," bbox x",W[:,0].min().round(3),W[:,0].max().round(3)," y",W[:,1].min().round(3),W[:,1].max().round(3)," z",W[:,2].min().round(3),W[:,2].max().round(3))
for lo in np.arange(W[:,2].min(),W[:,2].max(),0.01):
    m=(W[:,2]>=lo)&(W[:,2]<lo+0.01)
    if m.sum()<5: continue
    P=W[m]; print(f"wz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] cx={P[:,0].mean():.3f} cy={P[:,1].mean():.3f}")
for lo in np.arange(Y[:,2].min(),Y[:,2].max(),0.01):
    m=(Y[:,2]>=lo)&(Y[:,2]<lo+0.01)
    if m.sum()<3: continue
    P=Y[m]; print(f"yz={lo:.3f} n={m.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
# book: dark points
dark=(rgb.max(1)<60)&(xyz[:,2]>0.89)&(xyz[:,0]>-0.3)
D=xyz[dark]; print("dark n",len(D)," bbox x",D[:,0].min().round(3),D[:,0].max().round(3)," y",D[:,1].min().round(3),D[:,1].max().round(3)," z",D[:,2].min().round(3),D[:,2].max().round(3))
