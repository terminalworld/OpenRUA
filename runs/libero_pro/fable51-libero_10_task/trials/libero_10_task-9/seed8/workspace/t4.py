from lib import *
from scene import *
r = Robot("t4"); sc = Scene(r.node)
q0 = r.arm_q()
def R_look(zdir, xdir_hint=np.array([0,0,-1.])):
    z = np.asarray(zdir,float); z/=np.linalg.norm(z)
    x = xdir_hint - z*np.dot(xdir_hint,z); x/=np.linalg.norm(x)
    y = np.cross(z,x)
    return R_from_axes(x,y,z)
cands = [((0.0,0.05,1.06),(0,1,0)),((0.0,0.02,1.07),(0,1,0)),((0.05,0.0,1.07),(0,1,0)),((0.0,0.0,1.08),(0,1,0)),
         ((0.2,-0.1,1.03),(-0.406,0.914,0)),((0.2,-0.05,1.05),(-0.406,0.914,0)),((0.1,0.0,1.07),(-0.2,0.98,0))]
for pt, zd in cands:
    R = R_look(zd)
    s = r.ik(np.array(pt), R, seed=q0)
    if s is None: print(pt, zd, "IK none"); continue
    v,_ = sc.check(s); print(pt, zd, "q=",np.round(s,3), "valid",v, flush=True)
