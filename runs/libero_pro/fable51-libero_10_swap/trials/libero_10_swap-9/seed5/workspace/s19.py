import numpy as np, scene, sys
for cam in ['frontview','sideview','agentview']:
    try:
        D,C,K,R,t=scene.cam_model(cam)
    except SystemExit as e:
        print(cam,"fail",e); continue
    P=scene.to_world(D,K,R,t)
    X,Y,Z=P[...,0],P[...,1],P[...,2]
    ok=np.isfinite(Z)
    print(cam,"cam at",np.round(t,3))
    m=ok&(np.abs(X-0.01)<0.09)&(np.abs(Y-0.01)<0.09)&(Z>0.905)&(Z<1.1)
    if m.sum()==0: print(" no mug pts"); continue
    print(" n",m.sum(),"zmax",np.round(Z[m].max(),3),"x",np.round([X[m].min(),X[m].max()],3),"y",np.round([Y[m].min(),Y[m].max()],3))
    for lo,hi in [(0.92,0.95),(0.95,0.97),(0.97,0.98),(0.98,0.99),(0.99,1.0),(1.0,1.02)]:
        b=m&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
    # grey mug too
    g=ok&(np.abs(X-0.007)<0.09)&(np.abs(Y-0.338)<0.09)&(Z>0.905)&(Z<1.1)
    if g.sum(): print(" grey zmax",np.round(Z[g].max(),3),"x",np.round([X[g].min(),X[g].max()],3),"y",np.round([Y[g].min(),Y[g].max()],3))
