import numpy as np, subprocess
subprocess.run(["python3","tools/perception/cam_snap.py","/birdview/depth/image_raw"], capture_output=True)
d = np.load("birdview.npy"); z = 3.0 - d
B=(-0.057,0.207)
best=None
for zt_guess in np.arange(0.95, 1.25, 0.01):
    dist=3.0-zt_guess; u=int(320+B[1]*579.41/dist); v=int(240+(B[0]+0.2)*579.41/dist)
    sub=z[v-25:v+25, u-25:u+25]; mask=(np.abs(sub-zt_guess)<0.012)
    if mask.sum()>150 and (best is None or mask.sum()>best[0]): best=(mask.sum(), zt_guess, np.median(sub[mask]), u, v, sub, mask)
n,zg,zt,u,v,sub,mask=best; vs,us=np.nonzero(mask); dd=3.0-zt
print("pot top z", round(zt,3), "n", n, "world", round(((vs.mean()+v-25)-240)*dd/579.41-0.2,4), round(((us.mean()+u-25)-320)*dd/579.41,4))
