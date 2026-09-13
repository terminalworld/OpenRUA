import numpy as np
pts=[]; cols=[]
for c in ["birdview","frontview","agentview","sideview"]:
    d=np.load(f"{c}_cloud.npz"); xyz=d["xyz"]; rgb=d["rgb"].astype(int)
    m=(xyz[:,2]>0.885)&(xyz[:,2]<1.05)&(xyz[:,0]>-0.33)&(xyz[:,0]<0.0)&(xyz[:,1]>-0.40)&(xyz[:,1]<0.05)
    pts.append(xyz[m]); cols.append(rgb[m])
xyz=np.concatenate(pts); rgb=np.concatenate(cols)
# colour clusters: print a few samples of non-white points by y band
nw = rgb.min(1)<=120
for ylo in np.arange(-0.30,-0.10,0.02):
    m=nw&(xyz[:,1]>=ylo)&(xyz[:,1]<ylo+0.02)&(xyz[:,2]>0.895)
    if m.sum()==0: continue
    print(f"y[{ylo:.2f},{ylo+0.02:.2f}] n={m.sum():4d} meanrgb={rgb[m].mean(0).round(0)} z[{xyz[m,2].min():.3f},{xyz[m,2].max():.3f}] x[{xyz[m,0].min():.3f},{xyz[m,0].max():.3f}]")
# candidate handle: yellow-ish AND bright: R>170,G>140
yel=(rgb[:,0]>160)&(rgb[:,1]>130)&(rgb[:,2]<120)&(xyz[:,2]>0.893)
Y=xyz[yel]; print("strict yellow n",len(Y))
if len(Y):
    print("bbox x",Y[:,0].min().round(3),Y[:,0].max().round(3)," y",Y[:,1].min().round(3),Y[:,1].max().round(3)," z",Y[:,2].min().round(3),Y[:,2].max().round(3))
    for lo in np.arange(Y[:,0].min(),Y[:,0].max(),0.01):
        m=(Y[:,0]>=lo)&(Y[:,0]<lo+0.01)
        if m.sum()<3: continue
        P=Y[m]; print(f"x={lo:.3f} n={m.sum():4d} y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
