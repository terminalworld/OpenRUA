from lib import *
from scene import *
r = Robot("t5"); sc = Scene(r.node)
sc.publish(world_objects())
q0 = r.arm_q()
mx,my = -0.0985,-0.252
def R_pitch(phi):
    c,s=np.cos(phi),np.sin(phi)
    z=np.array([0,c,-s]); y=np.array([1.,0,0]); x=np.cross(y,z)
    return R_from_axes(x,y,z)
for deg in [0,10,15,20,25]:
    phi=np.radians(deg); R=R_pitch(phi); z=R[:,2]
    pad = np.array([mx,my,0.94])
    grasp = pad - 0.094*z
    pre = grasp - 0.07*z
    qg = r.ik(grasp,R,seed=q0,timeout=2.0)
    qp = None if qg is None else r.ik(pre,R,seed=qg,timeout=2.0)
    def fmt(q, allow_mug=False):
        if q is None: return "IK-none"
        v,c = sc.check(q, verbose=False)
        c = sorted({tuple(x) for x in c})
        if allow_mug: c=[x for x in c if not ('white_mug' in x and any(k in x[0]+x[1] for k in ['finger','hand']))]
        return f"valid={v and not c} contacts={c} q={np.round(q,3)}"
    print(f"pitch {deg}: grasp origin={np.round(grasp,3)} -> {fmt(qg, True)}")
    print(f"          pre   origin={np.round(pre,3)} -> {fmt(qp)}", flush=True)
