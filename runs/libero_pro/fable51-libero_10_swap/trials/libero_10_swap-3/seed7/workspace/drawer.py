import numpy as np, subprocess
subprocess.run(["python3","snaps/seg.py","birdview"],capture_output=True,timeout=120)
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.08)&(X<-0.01)&(Y>-0.30)&(Y<0.0)&np.isfinite(Z)&(Z>0.975)&(Z<0.995)
print('panel top pts', m.sum(), 'y range', Y[m].min().round(3) if m.sum() else None, Y[m].max().round(3) if m.sum() else None)
m2=(X>-0.08)&(X<-0.01)&(Y>-0.30)&(Y<0.0)&np.isfinite(Z)&(Z>0.915)&(Z<0.93)
print('drawer floor visible y range', Y[m2].min().round(3) if m2.sum() else None, Y[m2].max().round(3) if m2.sum() else None)
m3=(X>-0.20)&(X<-0.05)&(Y>-0.40)&(Y<-0.09)&np.isfinite(Z)&(Z>0.94)&(Z<0.98)
print('bowl-ish pts', m3.sum(), 'x',X[m3].min().round(3) if m3.sum() else None, X[m3].max().round(3) if m3.sum() else None,'y',Y[m3].min().round(3) if m3.sum() else None,Y[m3].max().round(3) if m3.sum() else None)
