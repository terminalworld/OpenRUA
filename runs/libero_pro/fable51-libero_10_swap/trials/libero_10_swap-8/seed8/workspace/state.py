import numpy as np
from rob import *
r=Rob("state")
P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
np.save("bird_now.npy",P)
def region(name,xr,yr,zmin=0.94):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)
    if m.sum()==0: print(name,"empty"); return
    Q=P[m]; print(f"{name}: n={m.sum()} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] zmax={Q[:,2].max():.3f} top-pt={Q[Q[:,2].argmax()].round(3)}")
    for z in np.arange(zmin,Q[:,2].max(),0.02):
        mm=(Q[:,2]>z)&(Q[:,2]<z+0.02)
        if mm.sum(): print(f"   z{z:.2f}: n={mm.sum()} x[{Q[mm,0].min():.3f},{Q[mm,0].max():.3f}] y[{Q[mm,1].min():.3f},{Q[mm,1].max():.3f}]")
region("B", (-0.2,0.05),(0.12,0.35))
region("stove",(0.06,0.30),(-0.08,0.15),0.94)
region("A_orig",(-0.3,-0.1),(-0.3,-0.1))
print("tcp",r.tcp()[0].round(3),"fingers",r.fingers())
