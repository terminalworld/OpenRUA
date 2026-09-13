import numpy as np, sys
from rob import *
r=Rob("park")
cands=[[0.31,-0.3,-0.06,-2.4,-2.13,2.16,0.12],[0.31,-0.6,-0.06,-2.6,-2.13,2.0,0.12],[0.0,-0.8,0.0,-2.5,0.0,1.7,0.785],[0.31,-0.6,-0.06,-2.6,-2.13,2.5,0.12]]
for q in cands:
    p,R=r.tcp(np.array(q)); zs={l:r.fk(np.array(q),l)[0][2] for l in ["panda_link4","panda_link6","panda_hand"]}
    print(q,"tcp",p.round(3),"hz",R[:,2].round(2),{k:round(v,3) for k,v in zs.items()})
if len(sys.argv)>1:
    q=np.array(cands[int(sys.argv[1])]); dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15)
    print(r.move_q(q,dur)); print("tcp",r.tcp()[0].round(3))
