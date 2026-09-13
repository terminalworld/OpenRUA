import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
print("table z", np.median(z[300:340,250:400]))
for v in range(158,212):
    print(v, " ".join("%.3f"%P[v,u,0] for u in (280,)), " ".join("%3d"%int(round((z[v,u]-0.88)*1000)) for u in range(214,302,3)))
print("u->y:", " ".join("%.3f"%P[180,u,1] for u in range(214,302,3)))
