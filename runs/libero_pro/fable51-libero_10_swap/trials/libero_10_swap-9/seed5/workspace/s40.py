import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
BAR=np.array([-0.132,-0.3815,1.003])
for deg in [30,35]:
    for fx in [False,True]:
        Q=T(deg,fx); ok=[]; seed=a.arm_q()
        for name,p in [("above",BAR+[0,0,0.06]),("mid",BAR+[0,0,0.03]),("bar",BAR),("push",BAR+[0,0.05,0]),("up",BAR+[0,0.05,0.05])]:
            q=a.ik(p,Q,at_tcp=True,seed=seed,timeout=8)
            ok.append(q is not None)
            if q is not None: seed=q
        print(deg,fx,ok,flush=True)
