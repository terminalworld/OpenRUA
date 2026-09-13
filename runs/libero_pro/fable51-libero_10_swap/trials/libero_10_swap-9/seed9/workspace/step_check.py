from rob import *
r = Robot('chk')
q_side = quat_from_axes([0,0,1],[1,0,0],[0,1,0])   # hand z -> +y, fingers along x, hand x up
q_side2 = quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])
for name, qs in [('x-up', q_side), ('x-down', q_side2)]:
    print('====', name, np.round(qs,4))
    poses = {
     'pregrasp': ([-0.030, -0.22, 0.955], qs),
     'grasp':    ([-0.030, -0.133, 0.955], qs),
     'lift':     ([-0.030, -0.133, 1.05], qs),
     'pre_mw':   ([-0.155, -0.52, 1.00], qs),
     'pre_mw2':  ([-0.155, -0.48, 1.00], qs),
     'in_mw':    ([-0.155, -0.403, 0.99], qs),
    }
    seed = r.arm_q()
    for k,(p,q) in poses.items():
        sol = r.ik(p, q, seed=seed)
        print(k, None if sol is None else np.round(sol,3))
        if sol is not None:
            fp, fq = r.fk(sol); print('   fk', np.round(fp,3), np.round(fq,3))
            seed = sol
