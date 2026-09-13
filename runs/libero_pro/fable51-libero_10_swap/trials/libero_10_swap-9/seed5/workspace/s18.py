import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
print("cam t",np.round(t,3))
for lo,hi in [(0.93,0.95),(0.95,0.97),(0.97,0.98),(0.98,0.99),(0.99,0.999),(0.999,1.01)]:
    b=ok&(Z>lo)&(Z<hi)&(np.hypot(X-0.01,Y-0.01)<0.1)
    if b.sum()==0: continue
    r=np.hypot(X[b]-0.01,Y[b]-0.01)
    ang=np.degrees(np.arctan2(Y[b]-0.01,X[b]-0.01))
    print(f"z {lo}-{hi} n={b.sum()} r pct {np.round(np.percentile(r,[5,50,95]),3)} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# unique depth values near 0.25
d=D[np.isfinite(D)]
u,c=np.unique(np.round(d,4),return_counts=True)
print("most common depths",[(float(a),int(b)) for a,b in sorted(zip(u,c),key=lambda x:-x[1])[:5]], "dmax",d.max())
