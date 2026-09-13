import numpy as np, rob, sys, subprocess
P=np.load("barpinch_pose.npy"); H=P[:3]; quat=list(P[3:7]); zh=P[7:10]
r=rob.Robot(); step=sys.argv[1]
if step=="pre":
    r.gripper(0.04); print("fingers",r.fingers())
    q_up=[-0.563,1.112,-0.116,-0.752,-0.007,1.282,1.365]
    q_up=r.ik(list(H+np.array([0,0,0.08])),quat,seed=q_up,timeout=5,avoid=True); print("q_up",np.round(q_up,3))
    ok=r.plan_exec(list(q_up),vel=0.3,acc=0.3,planning_time=10.0,time_scale=2.0)
    print("plan_exec:",ok,"q now",np.round(r.arm_q(),3)); p,_=r.fk(); print("hand",np.round(p,3),"target",np.round(H+[0,0,0.08],3))
elif step=="down":
    wps=[(list(H+np.array([0,0,dz])),quat) for dz in (0.05,0.03,0.015,0.0)]
    qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.0,max_jump=0.5,avoid=False)
    print("cart ok" if qs is not None else "cart FAILED"); p,qq=r.fk(); print("hand",np.round(p,3),"target",np.round(H,3),"quat err",np.round(np.abs(np.array(qq)-np.array(quat)).max(),3))
elif step=="close":
    r.gripper(0.0); import time
    for i in range(5): r.spin(0.2)
    print("fingers",r.fingers())
