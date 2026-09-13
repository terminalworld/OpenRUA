import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
P = np.load("snaps/P_front.npy")
for u in (272, 278, 284, 302):
    print("u",u)
    for v in range(352, 368): print(v, P[v,u].round(3))
S = np.load("snaps/P_side.npy")
# side: handle seen end-on; find pixels with y in [-0.12,-0.03], x in [-0.09,-0.04], z<1.2
m = (S[...,0]>-0.10)&(S[...,0]<-0.03)&(S[...,1]>-0.13)&(S[...,1]<-0.02)&(S[...,2]<1.1)&(S[...,2]>0.905)
pts = S[m]; print("side handle pts", pts.shape, "z range", pts[:,2].min(), pts[:,2].max(), "x range", pts[:,0].min(), pts[:,0].max())
import collections
hist = np.histogram(pts[:,2], bins=np.arange(0.90,0.98,0.005)); print(hist)
