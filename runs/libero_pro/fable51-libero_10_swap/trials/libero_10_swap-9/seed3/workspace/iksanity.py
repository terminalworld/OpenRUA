import numpy as np, rob
r=rob.Robot(); p,q=r.fk(); q0=r.arm_q(); print("q0",np.round(q0,3),"p",np.round(p,3))
for dz in (0,-0.01,-0.03):
    pos=list(np.array(p)+[0,0,dz])
    try: s=r.ik(pos,q,seed=q0,timeout=1.0,avoid=False)
    except Exception as e: s="EXC "+str(e)[:80]
    print(dz, None if s is None else np.round(s,3) if not isinstance(s,str) else s)
    try: s=r.ik(pos,q,seed=q0,timeout=1.0,avoid=True)
    except Exception as e: s="EXC "+str(e)[:80]
    print(" avoid", None if s is None else np.round(s,3) if not isinstance(s,str) else s)
