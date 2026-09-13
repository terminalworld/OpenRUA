from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
YG=167.3
Gk=np.array([-0.309,-0.168,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
Rr=np.array([0.0,-0.2,1.17]); PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
best=None; n_ok=0
for p in (-10,0,-20):
    for i in range(24):
        seed=nice_seed(Gk)+rng.normal(0,0.5,7)
        s=r.ik_world(Gk,Q(p,YG),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        try:
            hov=path_ik(r,Gk,Hk,Q(p,YG),Q(p,YG),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,YG),Q(p,YG),3,s,m=0.15)
            seg1b=path_ik(r,Lf,M,Q(p,YG),Q(p,YG),6,seg1[-1],m=0.15,maxjump=0.5)
        except RuntimeError as e:
            continue
        for dj in (0.0,math.pi,-math.pi):
            q3=seg1b[-1].copy(); q3[6]+=dj
            if not margin_ok(q3,0.15): continue
            for ye in (45,90,0,135):
                try:
                    seg2=path_ik(r,M,Rr,Q(p,YG),Q(p,YG),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(p,YG),Q(PE,ye),16,seg2[-1],m=0.15,maxjump=0.5)
                    Xh=quat_to_R(*Q(PE,ye))[:,0]
                    Al=np.array([0.141,-0.004,0.941])+0.15*Xh; Ah=Al+[0,0,0.04]
                    seg4=path_ik(r,Rr,Ah,Q(PE,ye),Q(PE,ye),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,ye),Q(PE,ye),2,seg4[-1],m=0.15,maxjump=0.5)
                except RuntimeError as e:
                    continue
                allq=np.array(list(hov)+list(seg1)+list(seg1b)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
                n_ok+=1
                print('OK p',p,'dj',round(dj,2),'ye',ye,'start',np.round(s,2),'margin',round(mg,2),flush=True)
                if best is None or mg>best[0]: best=(mg,p,dj,ye,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,dj,ye,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan4.npz',p=p,ye=ye,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),seg1b=np.array(seg1b),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',round(mg,3),p,dj,ye)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('M',seg1b[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
