from pp import *
import time
pp_=PP(); r=pp_.r
q0=r.arm_q()
t=time.time(); s=r.ik_tcp([-0.042,0.254,0.938],R_G,seed=q0,timeout=0.5); print('call1',time.time()-t, s is not None)
t=time.time(); s=r.ik_tcp([0.5,0.5,0.5],R_G,seed=q0,timeout=0.5); print('fail call',time.time()-t, s is not None)
t=time.time(); s=r.ik_tcp([0.5,0.5,0.5],R_G,seed=q0,timeout=0.1); print('fail call 0.1',time.time()-t, s is not None)
