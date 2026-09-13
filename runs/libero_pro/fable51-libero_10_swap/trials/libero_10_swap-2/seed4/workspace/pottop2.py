import numpy as np, subprocess
subprocess.run(["python3","tools/perception/cam_snap.py","/birdview/depth/image_raw"], capture_output=True)
d = np.load("birdview.npy"); z = 3.0 - d
P=(-0.0585,0.19); zt=1.07; dist=3.0-zt
u=int(320+P[1]*579.41/dist); v=int(240+(P[0]+0.2)*579.41/dist)
sub=z[v-8:v+9, u-8:u+9]
print("pot top region z: max", np.round(sub.max(),3), "median", np.round(np.median(sub),3), "p90", np.round(np.percentile(sub,90),3))
