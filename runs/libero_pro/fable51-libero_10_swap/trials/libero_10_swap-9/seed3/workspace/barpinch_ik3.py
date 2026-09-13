import numpy as np, handcheck as h, rob, pickle
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
B=c+(-0.0075)*a+0.083*rho
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(2)
for gamma,beta,sgn in [(-20,-40,1),(-20,-40,-1),(-20,-35,-1),(-20,-35,1),(-25,-40,1),(-25,-40,-1),(-20,-45,1),(-20,-45,-1)]:
    g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh); yh=sgn*yh
    H=B+0.006*zh-0.1034*zh; quat=rob.quat_from_axes(zh,yh)
    best=None
    for k in range(12):
        seed=list(rng.uniform(lo,hi))
        if k==0: seed=[-0.62,1.08,0.04,-1.01,-0.18,1.59,1.44*sgn]
        try: q=r.ik(list(H),quat,seed=seed,timeout=0.5,avoid=True)
        except Exception: q=None
        if q is not None:
            q=np.array(q); marg=np.minimum(q-lo,hi-q).min()
            if -1.8<q[0]<0 and (best is None or marg>best[1]): best=(np.round(q,3),round(marg,2))
    print(gamma,beta,sgn,np.round(H,3),best,flush=True)
