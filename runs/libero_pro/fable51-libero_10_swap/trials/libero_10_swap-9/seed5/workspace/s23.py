import numpy as np, scene
for cam in ['frontview','agentview']:
    D,C,K,R,t=scene.cam_model(cam)
    P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
    m=ok&(np.abs(X-0.012)<0.09)&(Y>-0.05)&(Y<0.08)&(Z>0.905)&(Z<1.2)
    print(cam,"mug body n",m.sum(),"z",np.round([Z[m].min(),Z[m].max()],3))
    for lo,hi in [(0.93,0.96),(0.96,0.99),(0.99,1.02),(1.02,1.05),(1.05,1.08),(1.08,1.1)]:
        b=m&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}] cx={X[b].mean():.3f}")
