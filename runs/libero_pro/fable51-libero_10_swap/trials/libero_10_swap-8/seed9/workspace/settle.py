import numpy as np
from rob import *
r = Robot("settle")
home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785])
for _ in range(3):
    code, err = r.move_q(home, 4.0)
    if err < 0.01: break
print("final q", np.round(r.arm_q(),3), "fingers", np.round(r.fingers(),4))
d = r.depth("birdview")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
Z = P[:,2].reshape(480,640)
import cv2
mask = ((Z>1.04)&(Z<1.10)).astype(np.uint8); mask[:150,:]=0
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i][4] < 50: continue
    ys,xs = np.where(lab==i); pts = P.reshape(480,640,3)[ys,xs]
    print(f"pot lid: center xy {np.round(pts[:,:2].mean(0),4)} lid z {np.median(pts[:,2]):.4f}")
tab = P[(P[:,0]>-0.45)&(P[:,0]<0.0)&(P[:,1]>-0.4)&(P[:,1]<0.4)&(P[:,2]>0.95)&(P[:,2]<1.2)]
print("pts 5-30cm above table in the original pot area (x<0):", len(tab))
r.snap("agentview", "/workspace/agentview_final.png")
