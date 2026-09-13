import numpy as np
P=np.load("birdview_cloud.npy"); z=P[:,:,2]
for v in range(244,284,2):
    print(v, "%.3f"%P[v,330,0], " ".join("%3d"%int(round((z[v,u]-0.88)*100)) for u in range(300,352,2)))
print("u->y:", " ".join("%.3f"%P[262,u,1] for u in range(300,352,2)))
