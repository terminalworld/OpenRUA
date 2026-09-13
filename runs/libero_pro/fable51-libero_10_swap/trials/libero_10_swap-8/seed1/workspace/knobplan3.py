from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Gk=np.array([-0.397,-0.167,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
Rr=np.array([0.0,-0.2,1.17]); Ah=np.array([0.15,0.005,1.13]); Al=np.array([0.15,0.005,1.09])
PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
refs={(-10,180):np.array([-0.82,0.35,0.38,-2.6,2.37,3.21,1.08]),(10,0):np.array([-1.12,0.39,0.24,-2.61,-2.34,3.21,2.23]),(-20,180):None,(20,0):None}
best=None; n_ok=0
for (p,yaw),ref in refs.items():
    for i in range(40):
        seed=(ref if (ref is not None and i%2) else nice_seed(Gk))+rng.normal(0,0.5,7)
        s=r.ik_world(Gk,Q(p,yaw),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        yaw2=(yaw+180)%360
        try:
            hov=path_ik(r,Gk,Hk,Q(p,yaw),Q(p,yaw),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,yaw),Q(p,yaw),3,s,m=0.15)
            seg1b=path_ik(r,Lf,M,Q(p,yaw),Q(p,yaw),6,seg1[-1],m=0.15,maxjump=0.5)
            ok=False
            for dj in (math.pi,-math.pi):
                q3=seg1b[-1].copy(); q3[6]+=dj
                if not margin_ok(q3,0.15): continue
                try:
                    seg2=path_ik(r,M,Rr,Q(-p,yaw2),Q(-p,yaw2),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(-p,yaw2),Q(PE,yaw2+45),14,seg2[-1],m=0.15,maxjump=0.5)
                    seg4=path_ik(r,Rr,Ah,Q(PE,yaw2+45),Q(PE,yaw2+45),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,yaw2+45),Q(PE,yaw2+45),2,seg4[-1],m=0.15,maxjump=0.5)
                    ok=True; break
                except RuntimeError as e: last=str(e)
            if not ok: raise RuntimeError('no spin/rot '+last[:60])
        except RuntimeError as e:
            print('  fail',p,yaw,str(e)[:80]); continue
        allq=np.array(list(hov)+list(seg1)+list(seg1b)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
        n_ok+=1
        print('OK p',p,'yaw',yaw,'dj',round(dj,2),'start',np.round(s,2),'margin',round(mg,2),flush=True)
        if best is None or mg>best[0]: best=(mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan3.npz',p=p,yaw=yaw,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),seg1b=np.array(seg1b),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',round(mg,3),p,yaw,dj)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('M',seg1b[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
