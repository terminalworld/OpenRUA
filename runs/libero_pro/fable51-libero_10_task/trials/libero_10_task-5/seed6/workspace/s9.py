import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s9")
tip=np.array([MUG_C[0],MUG_C[1]+0.0485,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
A=[0.04,-0.86,0.52,-2.92,-2.86,3.68,-2.07]
wp=[(1.16,-31),(1.13,-28),(1.10,-25),(1.08,-22),(1.06,-18),(1.04,-14),(1.03,-12),(1.02,-10),(1.01,-8),(1.00,-6),(0.99,-4),(0.985,-3),(0.978,-1),(0.972,0)]
seed=A; chain=[]
for z,deg in wp:
    pos,q=pose(z,deg); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("IK fail",z,deg); break
    print("z %.3f tilt %3d origin %s delta %.2f %s"%(z,deg,pos.round(3),np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
json.dump(chain,open("chain2.json","w"))
