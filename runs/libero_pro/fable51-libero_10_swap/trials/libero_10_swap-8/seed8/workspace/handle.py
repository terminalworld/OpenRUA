import numpy as np
A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
for f in ("bird_world.npy","side_world.npy"):
    P=np.load(f).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    for name,c in (("A",A),("B",B)):
        d=P[:,:2]-c; r=np.linalg.norm(d,axis=1); az=np.degrees(np.arctan2(d[:,1],d[:,0]))
        m=(r<0.12)&(P[:,2]>0.90)
        print(f,name,m.sum())
        # for az bins of 15 deg and z bins of 1cm print max r
        zs=np.arange(0.90,1.07,0.01)
        print("      "+" ".join(f"{a:4d}" for a in range(-180,180,15)))
        for z0 in zs:
            row=[]
            for a0 in range(-180,180,15):
                mm=m&(P[:,2]>=z0)&(P[:,2]<z0+0.01)&(az>=a0)&(az<a0+15)
                row.append(f"{(r[mm].max()*100 if mm.any() else 0):4.1f}")
            print(f"{z0:.2f} "+" ".join(row))
