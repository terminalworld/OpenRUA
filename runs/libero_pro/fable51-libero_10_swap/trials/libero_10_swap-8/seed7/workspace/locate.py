import numpy as np, cv2, subprocess, sys
subprocess.run(["timeout","60","python3","tools/perception/cam_snap.py","/birdview/depth/image_raw","snaps/bird_now.png"],check=True,capture_output=True)
d = np.load("snaps/bird_now.npy"); fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
wx = -0.2 + (vs-cy)*d/fy; wy = (us-cx)*d/fx; wz = 3.0 - d
table=0.899
mask = ((wz > table+0.008) & (wz < table+0.5)).astype(np.uint8); mask[:150,:]=0
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4] < 20: continue
    m = lab==i
    top = m & (wz > wz[m].max()-0.03)   # topmost 3 cm (knob/lid) -> body centre
    print(f"comp area={stats[i,4]:5d} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] ztop={wz[m].max():.3f}  top-centre=({wx[top].mean():.3f},{wy[top].mean():.3f})")
