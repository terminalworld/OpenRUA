import numpy as np
d=np.load("birdview_cloud.npz"); pc=d["pc"]; col=d["col"]
z=pc[...,2]
reg=np.isfinite(z)&(pc[...,0]>-0.32)&(pc[...,0]<-0.08)&(np.abs(pc[...,1])<0.15)&(z>0.44)&(z<0.62)
P=pc[reg]; C=col[reg]
print("n",reg.sum())
print("x",P[:,0].min().round(3),P[:,0].max().round(3),"y",P[:,1].min().round(3),P[:,1].max().round(3),"z",P[:,2].min().round(3),P[:,2].max().round(3))
# print occupancy grid 1cm
xs=np.arange(-0.32,-0.08,0.01); ys=np.arange(-0.15,0.15,0.01)
H,_,_=np.histogram2d(P[:,0],P[:,1],bins=[xs,ys])
print("     y:", " ".join(f"{y*100:+.0f}"[-2:] for y in ys[:-1]))
for i,x in enumerate(xs[:-1]):
    print(f"x={x:+.2f} "+"".join(" #" if H[i,j]>0 else " ." for j in range(len(ys)-1)))
# top rim only (z>0.55)
rim=reg&(z>0.555)
R=pc[rim]; print("rim n",rim.sum(),"x",R[:,0].min().round(3),R[:,0].max().round(3),"y",R[:,1].min().round(3),R[:,1].max().round(3),"ctr",R[:,0].mean().round(3),R[:,1].mean().round(3))
