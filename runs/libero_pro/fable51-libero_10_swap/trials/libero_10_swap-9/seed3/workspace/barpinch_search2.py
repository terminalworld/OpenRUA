import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho)
B=c+(-0.0075)*a+0.083*rho
res=[]
for gamma in range(-75,-14,5):
    g=np.radians(gamma)
    for beta in (0,10,20):
        b=np.radians(beta)
        yh=np.cos(b)*tau+np.sin(b)*rho
        zdir=-np.cos(g)*rho+np.sin(g)*a          # in loop plane
        zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
        for sgn in (1,-1):
            Rm=h.pose_from(zh,sgn*yh)
            for delta in (0.006,0.010,0.014):
                tip=B+delta*zh; H=tip-0.1034*zh
                worst=0; what=set(); cl_t=9; cl_b=9
                for gg in (0.040,0.005):
                    dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug,verbose=False)
                    worst=max(worst,dep.max()); what|=set(wh[dep>0])
                    cl=h.clearance(Pw,lab,c,a,perp); cl_t=min(cl_t,cl['table']); cl_b=min(cl_b,min(cl[k] for k in cl if k!='table'))
                wrist=H-0.1*zh
                res.append((round(worst,3),gamma,beta,sgn,delta,round(cl_t,3),round(cl_b,3),what,np.round(H,3),np.round(wrist,3)))
res.sort(key=lambda r:(r[0],-min(r[5],r[6])))
for r in res[:30]: print(r)
