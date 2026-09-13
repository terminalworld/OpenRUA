import numpy as np, scene
for cam in ['frontview','agentview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    # hand body: z in 1.245..1.30 near (0.022,-0.001)
    h=ok&(Z>1.25)&(Z<1.30)&(np.abs(X-0.022)<0.2)&(np.abs(Y)<0.1)
    print(cam,"hand slice n",h.sum(),"x",np.round([X[h].min(),X[h].max()],3),"y",np.round([Y[h].min(),Y[h].max()],3))
    # fingers: z 1.14..1.24
    f=ok&(Z>1.15)&(Z<1.23)&(np.abs(X-0.022)<0.2)&(np.abs(Y)<0.1)
    if f.sum(): print("  fingers x",np.round([X[f].min(),X[f].max()],3),"y",np.round([Y[f].min(),Y[f].max()],3))
    # mug base slices
    for lo,hi in [(0.902,0.91),(0.91,0.92),(0.92,0.93),(0.93,0.94)]:
        b=ok&(Z>lo)&(Z<hi)&(np.abs(X-0.012)<0.09)&(np.abs(Y-0.01)<0.09)
        if b.sum(): print(f"  base z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # handle: points with y<-0.045 near mug
    hd=ok&(Z>0.905)&(Z<1.02)&(np.abs(X-0.012)<0.09)&(Y<-0.045)&(Y>-0.1)
    if hd.sum():
        print("  handle n",hd.sum(),"x",np.round([X[hd].min(),X[hd].max()],3),"y",np.round([Y[hd].min(),Y[hd].max()],3),"z",np.round([Z[hd].min(),Z[hd].max()],3))
        for lo,hi in [(0.92,0.94),(0.94,0.96),(0.96,0.98),(0.98,1.0)]:
            s=hd&(Z>lo)&(Z<hi)
            if s.sum(): print(f"   handle z{lo}-{hi} n={s.sum()} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
