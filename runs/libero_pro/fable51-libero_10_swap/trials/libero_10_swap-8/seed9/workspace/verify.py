import numpy as np
from rob import *
r = Robot("verify")
# retreat to a neutral pose that clears the birdview of the stove
home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, 0.785])
r.move_q(home, 4.0)
for c in ["birdview","agentview","frontview","sideview"]:
    r.snap(c, f"/workspace/{c}_final.png")
d = r.depth("birdview")
fx=579.4112549695428; T=np.array([-0.2,0,3.0])
v,u = np.mgrid[0:480,0:640]
pc = np.stack([(u-320)*d/fx,(v-240)*d/fx,d],-1).reshape(-1,3)
P = np.stack([pc[:,1], pc[:,0], -pc[:,2]],-1) + T
tab = P[(P[:,0]>-0.4)&(P[:,0]<0.45)&(P[:,1]>-0.4)&(P[:,1]<0.4)]
plate = tab[(tab[:,2]>0.92)&(tab[:,2]<0.94)]
print("stove plate footprint x[%.3f,%.3f] y[%.3f,%.3f]" % (plate[:,0].min(), plate[:,0].max(), plate[:,1].min(), plate[:,1].max()))
import cv2
Z = P[:,2].reshape(480,640)
mask = ((Z>1.04)&(Z<1.10)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i][4] < 50: continue
    ys,xs = np.where(lab==i); pts = P.reshape(480,640,3)[ys,xs]
    lid = pts[(pts[:,2]>1.055)&(pts[:,2]<1.07)]
    print(f"pot lid blob: center xy {np.round(pts[:,:2].mean(0),4)} z lid median {np.median(pts[:,2]):.4f} zmax {pts[:,2].max():.4f} n={len(pts)}")
# anything left at table height above 0.95 outside the stove region? (pots still on table?)
rest = tab[(tab[:,2]>0.95)&~((tab[:,0]>0.0)&(tab[:,0]<0.31)&(tab[:,1]>-0.07)&(tab[:,1]<0.15))]
print("objects >5cm above table outside stove region:", len(rest), "pts", np.round(rest[:,:2].mean(0),3) if len(rest) else "")
