from ctl import *
c = Ctl("t8"); r=c.r
for deg in [0,10,15,20]:
    al=np.radians(deg)
    z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, -1., 0]); x = np.cross(y, z)
    R = R_from_axes(x, y, z)
    seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785]
    for tip in [[-0.04,-0.002,1.06],[-0.04,-0.002,0.955],[0.03,-0.002,0.955],[0.11,-0.002,0.955]]:
        o=np.array(tip)-0.1034*z
        q=c.ik_valid(o,R,seed=seed,tries=8,ignore=("yellow",))
        print(deg, np.round(o,3), None if q is None else np.round(q,3), flush=True)
        if q is not None: seed=q
