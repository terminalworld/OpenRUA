import numpy as np, scene
for cam in ['birdview','frontview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    m=ok&(X>-0.25)&(X<-0.02)&(Y>-0.52)&(Y<-0.37)&(Z>0.93)&(Z<1.08)
    print(cam,"n",m.sum())
    if m.sum():
        print(" z",np.round([Z[m].min(),Z[m].max()],3))
        for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.08)]:
            b=m&(Z>lo)&(Z<hi)
            if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # microwave face: points with z in 1.0..1.08, x in opening, y near -0.36
    f=ok&(X>-0.30)&(X<0.08)&(Y>-0.40)&(Y<-0.30)&(Z>1.095)&(Z<1.12)
    if f.sum(): print(" top front edge y min", np.round(Y[f].min(),3), "x", np.round([X[f].min(),X[f].max()],3))
