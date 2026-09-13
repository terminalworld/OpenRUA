import numpy as np
for cam in ['frontview','sideview','birdview']:
    W=np.load(f'{cam}_world.npy')
    for name,(x0,x1,y0,y1) in {'pot1':(-0.30,-0.10,-0.32,-0.10),'pot2':(-0.14,0.04,0.13,0.35),'stove':(0.09,0.33,-0.09,0.15),'knob':(0.0,0.11,-0.03,0.09)}.items():
        m=(W[...,0]>x0)&(W[...,0]<x1)&(W[...,1]>y0)&(W[...,1]<y1)&(W[...,2]>0.905)&(W[...,2]<1.2)
        P=W[m]
        if len(P)<10: continue
        print(cam,name,len(P))
        for zl in np.arange(0.91,1.07,0.01):
            s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.01)]
            if len(s): print(f'  z={zl:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
