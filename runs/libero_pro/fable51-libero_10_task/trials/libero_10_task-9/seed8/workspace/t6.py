from lib import *
from scene import *
r = Robot("t6"); sc = Scene(r.node)
q0 = r.arm_q()
mx,my = -0.0985,-0.252
def R_py(phi, psi):
    # approach dir: horizontal (-sin psi, cos psi) tilted down by phi
    c,s=np.cos(phi),np.sin(phi)
    h = np.array([-np.sin(psi), np.cos(psi), 0.])
    z = c*h + np.array([0,0,-s])
    y = np.array([np.cos(psi), np.sin(psi), 0.])
    x = np.cross(y,z)
    return R_from_axes(x,y,z)
def fmt(q, allow_mug=False):
    if q is None: return "IK-none"
    v,c = sc.check(q, verbose=False)
    c = sorted({tuple(x) for x in c})
    if allow_mug: c=[x for x in c if not ('white_mug' in x and any(k in x[0]+x[1] for k in ['finger','hand']))]
    return f"ok={not c} contacts={c} q={np.round(q,3)}"
for deg,yaw in [(10,0),(12,0),(15,0),(10,15),(10,25),(15,20)]:
    R=R_py(np.radians(deg), np.radians(yaw)); z=R[:,2]
    pad=np.array([mx,my,0.94]); grasp=pad-0.094*z
    qg=r.ik(grasp,R,seed=q0)
    print(f"pitch {deg} yaw {yaw}: grasp {np.round(grasp,3)} -> {fmt(qg,True)}", flush=True)
    if qg is None: continue
    for back in [0.05,0.06]:
        pre=grasp-back*z; qp=r.ik(pre,R,seed=qg)
        print(f"     pre back={back} {np.round(pre,3)} -> {fmt(qp)}", flush=True)
