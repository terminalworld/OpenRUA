import numpy as np
d=np.load("final_bird_depth.npy")
fx=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
reg=(wx>-0.12)&(wx<0.12)&(wy>0.0)&(wy<0.40)
# anything between table and cabinet top in front of the cabinet face?
for lo,hi in [(0.905,0.93),(0.93,0.99),(0.99,1.11)]:
    k=reg&(wz>lo)&(wz<hi)
    print(f"z {lo}-{hi}: n={k.sum()}", "y range", (round(wy[k].min(),3), round(wy[k].max(),3)) if k.sum() else None)
k=reg&(wz>0.93)&(wz<0.99)
if k.sum():
    for ylo in np.arange(0.18,0.26,0.01):
        kk=k&(wy>=ylo)&(wy<ylo+0.01)
        if kk.sum(): print(f"  y {ylo:.2f}: n={kk.sum()} z {wz[kk].min():.3f}-{wz[kk].max():.3f} x {wx[kk].min():.3f}..{wx[kk].max():.3f}")
# cabinet top edge (front) y
top=reg&(wz>1.11)&(wz<1.14)
print("cabinet top y min", round(wy[top].min(),4))
# bottle anywhere on the table outside cabinet? table region z 0.91-1.0 excluding cabinet footprint and bowl
tab=(wx>-0.6)&(wx<0.4)&(wy>-0.5)&(wy<0.5)&(wz>0.915)&(wz<1.0)&~((wy>0.19)&(wx>-0.13)&(wx<0.13))
print("objects on table (z .915-1.0) n=",tab.sum(), "x",np.round([wx[tab].min(),wx[tab].max()],3),"y",np.round([wy[tab].min(),wy[tab].max()],3))
print("--- points in front of cabinet, y 0..0.19, z 0.905-1.0")
k=(wx>-0.2)&(wx<0.2)&(wy>0.0)&(wy<0.19)&(wz>0.905)&(wz<1.0)
for ylo in np.arange(0.0,0.19,0.02):
    kk=k&(wy>=ylo)&(wy<ylo+0.02)
    if kk.sum(): print(f"  y {ylo:.2f}: n={kk.sum()} z {wz[kk].min():.3f}-{wz[kk].max():.3f} x {wx[kk].min():.3f}..{wx[kk].max():.3f}")
