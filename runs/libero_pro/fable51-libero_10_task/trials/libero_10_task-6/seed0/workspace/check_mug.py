import numpy as np
from scipy.optimize import least_squares
d=np.load("agentview_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
# red mug region near plate
reg=np.isfinite(z)&(np.abs(pc[...,0]-0.126)<0.12)&(np.abs(pc[...,1]-0.016)<0.12)&(z>0.47)
P=pc[reg]; print("mug pts",len(P),"z top",P[:,2].max().round(4))
for zl,zh in [(0.50,0.54),(0.54,0.58),(0.58,0.62)]:
    s=(P[:,2]>zl)&(P[:,2]<zh); Q=P[s][:,:2]
    if len(Q)<30: continue
    f=lambda c: np.hypot(Q[:,0]-c[0],Q[:,1]-c[1])-c[2]
    r=least_squares(f,[0.126,0.016,0.04])
    print(f"z[{zl},{zh}] n={len(Q)} center=({r.x[0]:.3f},{r.x[1]:.3f}) r={r.x[2]:.3f} rms={np.sqrt(np.mean(r.fun**2)):.4f}")
b=np.load("birdview_cloud.npz"); pcb=b["pc"]; zb=pcb[...,2]
m=np.isfinite(zb)&(np.abs(pcb[...,0]-0.126)<0.12)&(np.abs(pcb[...,1]-0.016)<0.12)&(zb>0.50)
B=pcb[m]; print("birdview mug: n",len(B),"x",B[:,0].min().round(3),B[:,0].max().round(3),"y",B[:,1].min().round(3),B[:,1].max().round(3),"z max",B[:,2].max().round(3), "rim ctr", B[B[:,2]>B[:,2].max()-0.015][:,:2].mean(0).round(3))
# plate check
m2=np.isfinite(zb)&(np.abs(pcb[...,0]-0.126)<0.12)&(np.abs(pcb[...,1]-0.016)<0.12)&(zb>0.44)&(zb<0.47)
Pl=pcb[m2]; print("plate pts x",Pl[:,0].min().round(3),Pl[:,0].max().round(3),"y",Pl[:,1].min().round(3),Pl[:,1].max().round(3),"ctr",((Pl[:,0].min()+Pl[:,0].max())/2).round(3),((Pl[:,1].min()+Pl[:,1].max())/2).round(3))
