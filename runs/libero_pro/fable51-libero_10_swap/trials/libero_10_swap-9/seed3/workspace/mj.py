import numpy as np, rob, sys, json
r=rob.Robot(); q=json.loads(sys.argv[1]); T=float(sys.argv[2]) if len(sys.argv)>2 else 4.0
print("before",np.round(r.arm_q(),3))
code,err=r.move_joints(q,T); print("code",code,"err",round(err,4)); print("after",np.round(r.arm_q(),3)); p,_=r.fk(); print("hand",np.round(p,3))
