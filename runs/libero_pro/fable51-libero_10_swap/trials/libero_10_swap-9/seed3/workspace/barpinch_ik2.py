import numpy as np, handcheck as h, rob, pickle
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
def pose(gamma,beta,delta,sgn):
    g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    tip=B+delta*zh; H=tip-0.1034*zh
    return H,zh,sgn*yh
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(1); found={}
for gamma in (-15,-10,-5,0):
  for beta in (-45,-40,-35,-30,-25):
    for sgn in (1,-1):
      H,zh,yh=pose(gamma,beta,0.006,sgn); quat=rob.quat_from_axes(zh,yh)
      sols=[]
      for k in range(10):
        seed=list(rng.uniform(lo,hi))
        try: q=r.ik(list(H),quat,seed=seed,timeout=0.5,avoid=True)
        except Exception: q=None
        if q is not None: sols.append(np.round(q,3))
      print(gamma,beta,sgn,"H",np.round(H,3),"sols",len(sols), sols[0] if sols else "",flush=True)
      if sols: found[(gamma,beta,sgn)]=(H,quat,sols)
pickle.dump(found,open("barpinch_sols.pkl","wb"))
