import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho)
B=c+(-0.0075)*a+0.083*rho    # bar centre
print("bar centre",np.round(B,3))
res=[]
for beta in range(0,61,5):
    b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho; zh=-(np.cos(b)*rho-np.sin(b)*tau)
    for sgn in (1,-1):
        Rm=h.pose_from(zh,sgn*yh)
        for delta in (0.004,0.008,0.012):
            for off in (-0.006,0.0,0.006):     # offset along y_h (closing dir) of hand centre from bar
                tip=B+delta*zh+off*yh
                H=tip-0.1034*zh
                worst=0; what=set()
                for g in (0.040,0.005):
                    dep,wh,Pw,lab=h.check(H,Rm,gap_half=g,mug=mug,verbose=False)
                    if dep.max()>worst: worst=dep.max()
                    what|=set(wh[dep>0])
                    cl=h.clearance(Pw,lab,c,a,perp)
                res.append((worst,beta,sgn,delta,off,round(cl['table'],3),round(min(cl[k] for k in cl if k!='table'),3),what,H))
res.sort(key=lambda r:(r[0],-r[5]))
for r in res[:25]: print(r[:8], np.round(r[8],3))
