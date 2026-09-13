import numpy as np
Z = np.load("birdZ.npy"); d=np.load("birdview_depth.npy")
fx=579.4112549695428; cx=320; cy=240; T=np.array([-0.2,0,3.0])
def w(u,v):
    z=d[v,u]; pc=np.array([(u-cx)*z/fx,(v-cy)*z/fx,z]); return np.array([pc[1],pc[0],-pc[2]])+T
np.set_printoptions(linewidth=250, precision=2)
for name,(x,y,ww,hh) in {"potA":(237,229,42,23),"potB":(365,275,43,25)}.items():
    sub = Z[y:y+hh, x:x+ww]
    print(name); print(((sub-0.9)*100).astype(int))
    ys,xs = np.where(sub > 1.04)
    cu, cv = x+xs.mean(), y+ys.mean()
    print(" top-region center px", cu, cv, "world", w(int(round(cu)),int(round(cv))))
    ys,xs = np.where(sub > 0.95)
    print(" body >5cm center px", x+xs.mean(), y+ys.mean(), "extent", xs.min(),xs.max(), ys.min(), ys.max())
print("between knob and plate", [round(float(Z[v,330]),3) for v in range(296,335)])
print("stove plate corners:", w(305,328), w(357,381))
