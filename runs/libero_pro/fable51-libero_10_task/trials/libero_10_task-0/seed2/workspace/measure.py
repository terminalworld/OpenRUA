from cloud import cloud
import numpy as np
B=cloud('birdview'); A=cloud('agentview')
def seg(P, xr, yr, zr, name):
    m=(P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])&(P[...,2]>zr[0])&(P[...,2]<zr[1])
    pts=P[m]
    if len(pts)==0: print(name,'none'); return
    c=pts[:,:2].mean(0)
    cov=np.cov((pts[:,:2]-c).T); w,v=np.linalg.eigh(cov)
    ax=v[:,1]; ang=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=(pts[:,:2]-c)@v
    print(f'{name}: n={len(pts)} center=({c[0]:.3f},{c[1]:.3f}) ztop={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} long-axis angle={ang:.1f} deg  extents long={proj[:,1].ptp():.3f} short={proj[:,0].ptp():.3f} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]')
seg(B,(0.0,0.2),(-0.3,-0.1),(0.44,0.5),'cream_cheese birdview')
seg(A,(0.0,0.2),(-0.3,-0.1),(0.44,0.5),'cream_cheese agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.48,0.53),'tomato can top agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.43,0.53),'tomato can all agentview')
seg(B,(-0.15,0.02),(-0.04,0.1),(0.43,0.6),'tomato can birdview')
seg(B,(-0.12,0.12),(0.12,0.4),(0.5,0.7),'basket birdview')
print('--- refined')
seg(B,(0.03,0.17),(-0.24,-0.14),(0.44,0.47),'cream_cheese birdview')
seg(A,(0.03,0.17),(-0.24,-0.14),(0.44,0.47),'cream_cheese agentview')
seg(A,(-0.15,0.02),(-0.04,0.1),(0.515,0.53),'tomato can topface agentview')
