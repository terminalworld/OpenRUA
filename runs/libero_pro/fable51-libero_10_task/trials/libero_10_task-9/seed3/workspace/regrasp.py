from rb import *
r=Robot('regrasp')
beta=np.radians(20)
zh=np.array([0,np.sin(beta),-np.cos(beta)]); yh=np.array([0,np.cos(beta),np.sin(beta)])
R=R_from(zh,yh)
def flange(t): return t-0.1034*zh
x_mug,y_mug,rim=-0.011,0.323,1.22
tcp=np.array([x_mug,y_mug-0.0435,rim-0.012])
def go(t,sec,seed=None):
    q=r.ik(flange(t),R,seed=seed)
    if q is None: raise SystemExit('IK fail '+str(t))
    code,err=r.move(q,sec)
    if err>0.01: code,err=r.move(q,3)
    p=r.fk()[0]; print('at tcp',(p+0.1034*zh).round(3),'target',t.round(3),code,err.round(4))
    return q
q=go(tcp+np.array([0,0,0.10]),5)
q=go(tcp+np.array([0,0,0.04]),3,q)
q=go(tcp,3,q)
print('wrench',r.wrench().round(2))
print('grip',r.grip(0.0))
r.spin(0.3); print('fingers',np.round(r.fingers(),4))
q=go(tcp+np.array([0,0,0.035]),3,q)
print('fingers after lift',np.round(r.fingers(),4),'wrench',r.wrench().round(2))
r.snap('agentview','agent_regrasp.png')
