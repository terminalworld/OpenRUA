import numpy as np, rob, sys
r=rob.Robot(); dz=float(sys.argv[1]); n=int(sys.argv[2]) if len(sys.argv)>2 else 3
p,q=r.fk(); print("hand",np.round(p,3),"fingers",r.fingers())
wps=[(list(p+np.array([0,0,dz*(i+1)/n])),list(q)) for i in range(n)]
qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.0,max_jump=0.5,avoid=False)
print("ok" if qs is not None else "FAILED","fingers",r.fingers())
