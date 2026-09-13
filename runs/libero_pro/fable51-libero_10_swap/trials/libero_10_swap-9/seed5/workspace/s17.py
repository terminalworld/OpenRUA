import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# find pixel closest to mug center (0.01,0.01) at rim level
ok=np.isfinite(Z)
m=ok&(Z>0.985)&(Z<1.02)
print("rim-level pts", m.sum(), "x",np.round([X[m].min(),X[m].max()],3),"y",np.round([Y[m].min(),Y[m].max()],3), "zmed",np.round(np.median(Z[m]),3))
# histogram of z for points within 0.06 of center
mm=ok&(np.hypot(X-0.01,Y-0.01)<0.07)&(Z>0.905)
h,e=np.histogram(Z[mm],bins=np.arange(0.90,1.03,0.005))
for c,lo in zip(h,e[:-1]): print(f"{lo:.3f} {c}")
# radial profile: for points with z>0.99 (rim), radius from center
r=np.hypot(X[m]-0.01,Y[m]-0.01)
print("rim radius pct", np.round(np.percentile(r,[5,50,95,99]),3))
# body: points z in 0.93..0.98
b=ok&(Z>0.93)&(Z<0.98)&(np.hypot(X-0.01,Y-0.01)<0.08)
r=np.hypot(X[b]-0.01,Y[b]-0.01); print("body radius pct", np.round(np.percentile(r,[5,50,95,99]),3), "n",b.sum())
