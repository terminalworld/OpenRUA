from rob import *
r = Robot('scan')
seed = r.arm_q()
center = np.array([-0.16,-0.27,1.0])
for psi in [15,30,45,60]:
    d = np.array([np.sin(np.radians(psi)), np.cos(np.radians(psi)), 0])
    hx = np.array([0,0,1.0]); hy = np.cross(d, hx)  # hand_y = z x x
    q = quat_from_axes(hx, hy, d)
    row=[]
    for s in [0.20,0.25,0.30,0.35,0.40,0.45,0.50,0.55]:
        p = center - s*d
        sol = r.ik(p, q, seed=seed, timeout=1.0, attempts=2)
        row.append(f's={s:.2f}@({p[0]:+.2f},{p[1]:+.2f}):' + ('ok' if sol is not None else '--'))
    print(f'psi={psi}', ' '.join(row), flush=True)
