import numpy as np, handcheck as h, rob
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
B=c+(-0.0075)*a+0.083*rho
def pose(gamma,beta,delta,sgn):
    g=np.radians(gamma); b=np.radians(beta)
    yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    Rm=h.pose_from(zh,sgn*yh); tip=B+delta*zh; H=tip-0.1034*zh
    return H,Rm,zh,sgn*yh
r=rob.Robot(); print("q now",np.round(r.arm_q(),2),"fingers",r.fingers())
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(0)
found={}
for gamma in (-20,-25,-30,-35):
  for beta in (-15,-20,-25):
    for sgn in (1,-1):
      H,Rm,zh,yh=pose(gamma,beta,0.008,sgn); quat=rob.quat_from_axes(zh,yh)
      sols=[]
      for k in range(12):
        seed=list(rng.uniform(lo,hi))
        try: q=r.ik(list(H),quat,seed=seed,timeout=1.0,avoid=True)
        except Exception as e: q=None
        if q is not None: sols.append(np.round(q,2))
      print(gamma,beta,sgn,"H",np.round(H,3),"sols",len(sols), sols[0] if sols else "")
      if sols: found[(gamma,beta,sgn)]=(H,quat,sols)
np.save("barpinch_sols.npy",np.array([(k,v[0],v[1],v[2]) for k,v in found.items()],dtype=object),allow_pickle=True)
