import numpy as np
W=np.load('frontview_world.npy')
for name,(x0,x1,y0,y1) in {'pot1':(-0.30,-0.10,-0.32,-0.10),'pot2':(-0.14,0.04,0.13,0.35)}.items():
    m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.905)&(W[...,2]<1.2)
    P=W[m]
    print(name)
    for zl in np.arange(0.905,1.06,0.005):
        s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.005)]
        if len(s):
            ys=np.sort(s[:,1])
            # cluster gaps
            print(f'  z={zl:.3f} n={len(s):3d} y[{ys[0]:.3f},{ys[-1]:.3f}] w={ys[-1]-ys[0]:.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]')
