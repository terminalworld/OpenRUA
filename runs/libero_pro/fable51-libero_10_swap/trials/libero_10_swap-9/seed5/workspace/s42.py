import scene, numpy as np
D,C,K,R,t=scene.cam_model("birdview")
P=scene.to_world(D,K,R,t); ok=np.isfinite(P[...,2])
pts=P[ok]
# door region: x in [-0.36,-0.26], y in [-0.65,-0.36], z>0.92
s=pts[(pts[:,0]>-0.36)&(pts[:,0]<-0.26)&(pts[:,1]>-0.66)&(pts[:,1]<-0.365)&(pts[:,2]>0.915)]
print("door pts",len(s),"x",np.round([s[:,0].min(),s[:,0].max()],3),"y",np.round([s[:,1].min(),s[:,1].max()],3),"z",np.round([s[:,2].min(),np.percentile(s[:,2],50),s[:,2].max()],3))
for ylo in np.arange(-0.64,-0.36,0.04):
    q=s[(s[:,1]>=ylo)&(s[:,1]<ylo+0.04)]
    if len(q): print(f" y{ylo:.2f}: n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] ztop={np.percentile(q[:,2],95):.3f}")
# microwave top
m=pts[(pts[:,0]>-0.28)&(pts[:,0]<0.07)&(pts[:,1]>-0.35)&(pts[:,1]<-0.16)&(pts[:,2]>1.0)]
print("mw top z",np.round(np.percentile(m[:,2],[5,50,95]),3),"x",np.round([m[:,0].min(),m[:,0].max()],3),"y",np.round([m[:,1].min(),m[:,1].max()],3))
# face plane: points with y in [-0.40,-0.35], z 0.92-1.11, x in [-0.29,0.08]
f=pts[(pts[:,0]>-0.29)&(pts[:,0]<0.08)&(pts[:,1]>-0.42)&(pts[:,1]<-0.34)&(pts[:,2]>0.92)&(pts[:,2]<1.12)]
print("front region pts",len(f))
if len(f): print(" y",np.round(np.percentile(f[:,1],[1,50,99]),3),"z",np.round(np.percentile(f[:,2],[1,50,99]),3))
