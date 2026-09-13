import numpy as np
W=np.load('frontview_world.npy')
# pot1 handle side: y<-0.20 ; list clusters in y per z slice
m=(W[...,0]>-0.30)&(W[...,0]<-0.10)&(W[...,1]>-0.32)&(W[...,1]<-0.10)&(W[...,2]>0.94)&(W[...,2]<1.06)
P=W[m]
for zl in np.arange(0.94,1.06,0.005):
    s=P[(P[:,2]>=zl)&(P[:,2]<zl+0.005)]
    if len(s)==0: continue
    ys=np.sort(s[:,1]); gaps=np.where(np.diff(ys)>0.006)[0]
    segs=[]; start=0
    for g in gaps: segs.append((ys[start],ys[g])); start=g+1
    segs.append((ys[start],ys[-1]))
    print(f'z={zl:.3f} '+' | '.join(f'[{a:.3f},{b:.3f}]' for a,b in segs))
