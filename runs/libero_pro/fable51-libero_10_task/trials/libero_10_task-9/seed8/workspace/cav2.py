import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (X>-0.16)&(X<0.16)&(Y>0.29)&(Y<0.47)&(Z>0.905)&(Z<1.095)
xs,ys,zs = X[m],Y[m],Z[m]
print("interior pts", m.sum())
xb = np.arange(-0.16,0.16,0.02); yb=np.arange(0.29,0.47,0.02)
print("min z grid rows=x cols=y"); print("      "+" ".join(f"{y:5.2f}" for y in yb))
for x in xb:
    row=[]
    for y in yb:
        mm=(xs>=x)&(xs<x+0.02)&(ys>=y)&(ys<y+0.02)
        row.append(f"{zs[mm].min():5.3f}/{zs[mm].max():5.3f}" if mm.sum() else "     -     ")
    print(f"{x:5.2f} "+" ".join(row))
