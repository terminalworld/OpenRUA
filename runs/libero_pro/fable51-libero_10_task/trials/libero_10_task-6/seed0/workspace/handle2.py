import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pc=d["pc"]; col=d["col"]; z=pc[...,2]
m=np.isfinite(z)&(pc[...,0]>-0.26)&(pc[...,0]<-0.13)&(pc[...,1]<-0.026)&(pc[...,1]>-0.12)&(z>0.44)&(z<0.60)
P=pc[m]; C=col[m]; print("pts",m.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
# top-down occupancy 2.5mm in x,y for z>0.50 (handle top region)
for zl,zh in [(0.54,0.60),(0.50,0.54),(0.46,0.50)]:
    s=(P[:,2]>=zl)&(P[:,2]<zh); Q=P[s]
    if not len(Q): continue
    print(f"--- z[{zl},{zh}] n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]")
    xs=np.arange(-0.24,-0.14,0.005); ys=np.arange(-0.10,-0.02,0.005)
    H,_,_=np.histogram2d(Q[:,0],Q[:,1],bins=[xs,ys])
    print("        y:"+"".join(f"{abs(y)*1000:3.0f}"[-3:] for y in ys[:-1]))
    for i,x in enumerate(xs[:-1]): print(f"x={x:+.3f} "+"".join("  #" if H[i,j]>0 else "  ." for j in range(len(ys)-1)))
# body wall position on -y side at z 0.50-0.54: min y of body points near x=-0.195
b=np.isfinite(z)&(np.abs(pc[...,0]+0.195)<0.01)&(z>0.50)&(z<0.56)&(pc[...,1]<0.0)
print("body/handle pts near x=-0.195, y values sorted sample:", np.unique(np.round(pc[b][:,1],3))[:40])
