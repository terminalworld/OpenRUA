import sys, numpy as np
sys.argv=[sys.argv[0]]
import ctl
c=ctl.Ctl()
tests=[(-0.0589,0.0,0.6744,0),(-0.084,-0.162,0.60,0),(-0.084,-0.162,0.60,10),(-0.084,-0.162,0.60,-10),(-0.084,-0.162,0.60,45),(-0.084,-0.162,0.60,90),(-0.084,-0.162,0.65,0),(-0.084,-0.162,0.55,0)]
for x,y,z,yaw in tests:
    q=c.solve_ik((x,y,z),yaw)
    print((x,y,z,yaw),"->",None if q is None else np.round(q,3).tolist())
