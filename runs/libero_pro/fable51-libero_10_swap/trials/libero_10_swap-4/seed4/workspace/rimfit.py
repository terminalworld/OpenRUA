import numpy as np, json, sys
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
roi=list(map(int,sys.argv[4:8])) if len(sys.argv)>=8 else None  # u0 v0 u1 v1
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
zs=d
P=np.stack([(us-cx)*zs/fx,(vs-cy)*zs/fy,zs,np.ones_like(zs)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
m=(W[2]>zlo)&(W[2]<zhi)&np.isfinite(d)&(d>0.01)
if roi: 
    rm=np.zeros_like(m); rm[roi[1]:roi[3],roi[0]:roi[2]]=True; m&=rm
x=W[0][m]; y=W[1][m]
print('n',m.sum(),'x[%.3f,%.3f] y[%.3f,%.3f] zmean %.3f'%(x.min(),x.max(),y.min(),y.max(),W[2][m].mean()))
# algebraic circle fit
A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; xc,yc=c[0]/2,c[1]/2; R=np.sqrt(c[2]+xc**2+yc**2)
print('circle center (%.4f,%.4f) R=%.4f'%(xc,yc,R))
