import numpy as np, sys, json
from ctl import *
r=Robot("s4"); go="--go" in sys.argv
def pose(tip,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    return np.array(tip)-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
top=[-0.44,-0.69,-0.33,-2.82,-2.77,3.7,2.9]
wp=[(1.20,-35),(1.15,-30),(1.10,-25),(1.08,-22),(1.06,-18),(1.045,-15),(1.03,-12),(1.02,-10),(1.01,-8),(0.995,-4),(0.985,-2),(0.978,0)]
seed=top; chain=[]
for z,deg in wp:
    pos,q=pose([-0.508,-0.1025,z],deg)
    s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print("IK fail at",z,deg); sys.exit(1)
    print("z %.3f tilt %3d origin %s delta %.2f  %s"%(z,deg,pos.round(3),np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
json.dump(chain,open("chain.json","w"))
