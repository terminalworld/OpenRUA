from ctl import *
c = Ctl("t7"); r=c.r
al = np.radians(30)
z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, -1., 0]); x = np.cross(y, z)
R = R_from_axes(x, y, z)
p0,R0 = r.fk([0,-0.161,0,-2.44,0,2.23,0.785]); print("home hand", np.round(p0,3), np.round(R0,2))
seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785]
p,Rs=r.fk(seed); print("seed hand", np.round(p,3), np.round(Rs[:,2],2), np.round(Rs[:,1],2))
for tip in [[-0.04,-0.002,1.06],[-0.04,-0.002,0.955],[0.11,-0.002,0.955]]:
    o=np.array(tip)-0.1034*z
    q=c.ik_valid(o,R,seed=seed,tries=10,ignore=("yellow",))
    print(np.round(o,3), None if q is None else np.round(q,3))
    if q is not None: seed=q
