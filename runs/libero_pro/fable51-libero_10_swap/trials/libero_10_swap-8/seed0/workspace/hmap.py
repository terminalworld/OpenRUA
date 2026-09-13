import sys, numpy as np
P=np.load("birdview_world.npy")
x0,x1,y0,y1=[float(v) for v in sys.argv[1:5]]
v=np.isfinite(P[...,2])
sel=v&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.94)
pts=P[sel]
xs=np.arange(x0,x1,0.01); ys=np.arange(y0,y1,0.01)
print("     "+" ".join(f"{y*100:4.0f}" for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(pts[:,0]>=x)&(pts[:,0]<x+0.01)&(pts[:,1]>=y)&(pts[:,1]<y+0.01)
        row.append(f"{(pts[m,2].max()-0.9)*100:4.1f}" if m.any() else "   .")
    print(f"{x*100:4.0f} "+" ".join(row))
