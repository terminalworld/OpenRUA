import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
us=list(range(214,380,3))
print("u->y:   ", " ".join("%4d"%int(P[180,u,1]*1000) for u in us))
for v in range(158,212,1):
    print(v, "%.3f"%P[v,280,0], " ".join("%4d"%int(round((z[v,u]-0.88)*1000)) for u in us))
