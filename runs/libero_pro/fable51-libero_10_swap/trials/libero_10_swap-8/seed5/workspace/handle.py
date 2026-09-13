import numpy as np
P = np.load("eih_cloud.npy"); z = P[...,2]
m = (z>0.93)&(z<1.1)&(P[...,1]<-0.236)&(np.abs(P[...,0]+0.205)<0.05)
s = P[m]; print("handle pts", len(s))
for lo in np.arange(-0.275,-0.235,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f} z[{ss[:,2].min():.3f},{ss[:,2].max():.3f}]")
# spout side
m = (z>0.93)&(z<1.1)&(P[...,1]>-0.166)&(np.abs(P[...,0]+0.205)<0.05)
s = P[m]; print("spout pts", len(s))
for lo in np.arange(-0.166,-0.13,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f} z[{ss[:,2].min():.3f},{ss[:,2].max():.3f}]")
# body width per y slice at top
m = (z>1.02)&(np.abs(P[...,0]+0.205)<0.06)&(P[...,1]>-0.25)&(P[...,1]<-0.15)
s = P[m]
for lo in np.arange(-0.245,-0.155,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"top y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f}")
