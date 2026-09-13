import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.93)&(Z<1.13)&(X<-0.165)&(X>-0.5)&(Y>-0.2)&(Y<0.32)
xs,ys,zs=X[m],Y[m],Z[m]
h = np.array([-0.17,0.28])
dvec = np.stack([xs-h[0], ys-h[1]],1); r = np.linalg.norm(dvec,axis=1)
ang = np.degrees(np.arctan2(dvec[:,1],dvec[:,0]))
for r0 in np.arange(0.0,0.45,0.02):
    mm=(r>=r0)&(r<r0+0.02)
    if mm.sum(): print(f"r[{r0:.2f}] n={mm.sum():5d} ang[{np.percentile(ang[mm],5):.1f},{np.percentile(ang[mm],95):.1f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}] x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
