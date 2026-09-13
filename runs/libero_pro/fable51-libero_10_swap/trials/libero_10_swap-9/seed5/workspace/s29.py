import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
print("cam",np.round(t,3))
m=ok&(X>-0.20)&(X<-0.07)&(Y>-0.47)&(Y<-0.25)&(Z>0.93)&(Z<1.12)
print("mug-ish n",m.sum())
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# cavity plate visible?
p=ok&(X>-0.23)&(X<-0.04)&(Y>-0.36)&(Y<-0.16)&(Z>0.92)&(Z<0.96)
print("plate pts",p.sum(), "z med",np.round(np.median(Z[p]),3) if p.sum() else None, "y range",np.round([Y[p].min(),Y[p].max()],3) if p.sum() else None)
# lip / front face
f=ok&(X>-0.23)&(X<-0.04)&(Y>-0.40)&(Y<-0.33)&(Z>0.90)&(Z<0.95)
if f.sum(): print("lip region z",np.round([Z[f].min(),Z[f].max()],3),"y",np.round([Y[f].min(),Y[f].max()],3))
