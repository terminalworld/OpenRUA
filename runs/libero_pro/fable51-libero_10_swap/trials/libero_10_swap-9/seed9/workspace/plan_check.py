from rob import *
r = Robot('plan')
def Q(psi_deg):
    d = np.array([np.sin(np.radians(psi_deg)), np.cos(np.radians(psi_deg)), 0.0])
    hx = np.array([0,0,1.0]); hy = np.cross(d, hx)
    return quat_from_axes(hx, hy, d), d
BAR = np.array([-0.030, -0.038])   # handle outer bar centre (x,y)
PINCH_Z = 0.96
def hand_for_mug(cx, cy, psi, z):
    q, d = Q(psi); c = np.array([cx, cy, 0]); return c - 0.168*d + np.array([0,0,z]), q
wps = [
 ('pregrasp', [-0.030, -0.235, PINCH_Z], Q(0)[0]),
 ('grasp',    [-0.030, -0.131, PINCH_Z], Q(0)[0]),
 ('lift',     [-0.030, -0.131, 1.03], Q(0)[0]),
 ('high',     [-0.030, -0.131, 1.25], Q(0)[0]),
]
for name,(cx,cy,psi,z) in [('over_a',(-0.16,-0.43,45,1.25)),('pre_ins',(-0.16,-0.43,45,1.006)),
                            ('ins1',(-0.16,-0.38,35,1.006)),('ins2',(-0.16,-0.33,22,1.006)),
                            ('ins3',(-0.16,-0.29,10,1.006)),('ins4',(-0.16,-0.25,0,1.006)),('ins5',(-0.16,-0.23,0,1.006)),
                            ('lower',(-0.16,-0.23,0,0.997)),('retreat',(-0.16,-0.33,0,0.997))]:
    p,q = hand_for_mug(cx,cy,psi,z); wps.append((name, p, q))
seed = r.arm_q(); sols = {}
for name,p,q in wps:
    sol = r.ik(p,q,seed=seed, timeout=2.0)
    print(f'{name:9s} hand {np.round(p,3)} ->', None if sol is None else np.round(sol,3), flush=True)
    if sol is not None: seed = sol; sols[name]=sol
np.save('plan_sols.npy', sols, allow_pickle=True)
