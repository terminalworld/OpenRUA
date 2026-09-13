from robot import *
from plan3 import obstacles, check_path
r=Robot()
plan=np.load('plan3.npy',allow_pickle=True).item()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); q1=plan['descend'][0]
qstop=np.array([1.459,0.701,-0.963,-2.845,-1.468,1.889,-0.582])
for t in np.linspace(0,1,21):
    q=q0+(q1-q0)*t
    fk=fk_links(r,q); p,R=fk['panda_hand']; tcp=p+TCP_OFF*R[:,2]
    hp=hand_points(p,R,0.04)
    lc=link_clearance(r,q)
    print(f't={t:.2f} tcp {np.round(tcp,3)} hand zmin {hp[:,2].min():.3f} hits {obstacles(hp)} minlink {min(c for _,c,_ in lc):.3f}')
print('stop tcp',np.round(r.tcp(qstop)[0],3))
