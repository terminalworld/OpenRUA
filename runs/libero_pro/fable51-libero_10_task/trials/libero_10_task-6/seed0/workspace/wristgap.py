import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pc=d["pc"]; col=d["col"]; dep=d["dep"]; z=pc[...,2]
print("depth range", np.nanmin(dep), np.nanmax(dep), "K", d["K"][0,0], d["K"][0,2], d["K"][1,2])
m=np.isfinite(z)&(z>0.50)&(z<0.70)&(np.abs(pc[...,1]-0.016)<0.08)&(pc[...,0]>-0.32)&(pc[...,0]<-0.06)
P=pc[m]; C=col[m]
gray=(np.abs(C[:,0].astype(int)-C[:,2].astype(int))<12)&(C[:,2]<120)
G=P[gray]; R=P[~gray]
print("gray n",gray.sum(),"other n",(~gray).sum())
xs=np.arange(-0.30,-0.08,0.005)
for i in range(len(xs)-1):
    g=G[(G[:,0]>=xs[i])&(G[:,0]<xs[i+1])]; o=R[(R[:,0]>=xs[i])&(R[:,0]<xs[i+1])]
    if len(g)+len(o)>3:
        print(f"x={xs[i]:+.3f} gray n={len(g):4d} z[{g[:,2].min() if len(g) else 0:.3f},{g[:,2].max() if len(g) else 0:.3f}] | other n={len(o):4d} z[{o[:,2].min() if len(o) else 0:.3f},{o[:,2].max() if len(o) else 0:.3f}] y[{o[:,1].min() if len(o) else 0:.3f},{o[:,1].max() if len(o) else 0:.3f}]")
