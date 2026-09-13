import numpy as np, json, sys
cam=sys.argv[1]; zlo,zhi,xlo,xhi,ylo,yhi=map(float,sys.argv[2:8])
meta=json.load(open(f'{cam}_meta.json')); d=np.load(f'{cam}_depth.npy')
T=np.array(meta['T']); K=np.array(meta['K']).reshape(3,3); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:d.shape[0],0:d.shape[1]]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)]).reshape(4,-1)
W=(T@P)[:3].reshape(3,*d.shape)
m=(W[2]>zlo)&(W[2]<zhi)&(W[0]>xlo)&(W[0]<xhi)&(W[1]>ylo)&(W[1]<yhi)&np.isfinite(d)&(d>0.01)
x=W[0][m]; y=W[1][m]
print('n',m.sum(),'x[%.4f,%.4f] y[%.4f,%.4f] z[%.3f,%.3f] bbox center (%.4f,%.4f)'%(x.min(),x.max(),y.min(),y.max(),W[2][m].min(),W[2][m].max(),(x.min()+x.max())/2,(y.min()+y.max())/2))
# outermost points: convex-ish: take top-most z pixels only (rim)
top=W[2][m].max(); mm=m&(W[2]>top-0.008); x2=W[0][mm]; y2=W[1][mm]
print('rim-top n',mm.sum(),'x[%.4f,%.4f] y[%.4f,%.4f] center (%.4f,%.4f)'%(x2.min(),x2.max(),y2.min(),y2.max(),(x2.min()+x2.max())/2,(y2.min()+y2.max())/2))
