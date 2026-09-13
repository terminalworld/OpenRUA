import numpy as np, subprocess
subprocess.run(["python3","cloud.py","birdview","bird.npz"],check=True,capture_output=True)
d=np.load('bird.npz'); X,Y,Z=d['X'],d['Y'],d['Z']
m=np.isfinite(Z)&(Y<-0.36)&(Y>-0.70)&(Z>0.92)&(Z<1.12)&(X>-0.40)&(X<0.1)
p=np.stack([X[m],Y[m],Z[m]],1)
print('door/handle pts',m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3))
for lo in np.arange(-0.66,-0.36,0.02):
    s=m&(Y>=lo)&(Y<lo+0.02)
    if s.sum()>2: print(f"y {lo:+.3f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] zmax={Z[s].max():.3f}")
# handle bar: points with x < panel inner face - 0.01 ; panel ~ x -0.259..-0.278 when open 90
hb=m&(X<-0.283)
if hb.sum(): 
    q=np.stack([X[hb],Y[hb],Z[hb]],1); print('handle(x<-0.283)',hb.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
