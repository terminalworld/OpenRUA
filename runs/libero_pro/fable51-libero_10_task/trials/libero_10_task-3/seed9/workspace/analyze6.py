import numpy as np
for name in ["agentview","sideview","birdview","frontview"]:
    try: P=np.load(f"snaps/{name}_xyz.npy").reshape(-1,3)
    except: continue
    m=(P[:,2]>0.905)&(P[:,2]<1.135)&(P[:,0]>-0.06)&(P[:,0]<0.06)&(P[:,1]>0.15)&(P[:,1]<0.30)
    Q=P[m]; print(name, len(Q))
    for ylo in np.arange(0.15,0.30,0.01):
        s=Q[(Q[:,1]>=ylo)&(Q[:,1]<ylo+0.01)]
        if len(s): print(f"  y {ylo:.2f}: n={len(s):4d} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]", np.histogram(s[:,2],bins=[0.9,0.94,0.98,1.02,1.06,1.10,1.14])[0])
