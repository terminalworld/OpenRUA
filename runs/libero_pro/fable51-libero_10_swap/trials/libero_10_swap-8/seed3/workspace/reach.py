from rob import *
r=Robot(); 
q=r.arm_q()
for z in [1.02,1.06,1.10]:
  for yaw in [0]:
    R=side_grasp_R(yaw,10)
    row=[]
    for x in [-0.1,0.0,0.07,0.13,0.21]:
        y=-0.217+(x+0.18)*(0.198/0.39)
        ok=any(r.ik_tcp([x,y,z],R,seed=s) is not None for s in [q]+SEEDS)
        row.append(f"{x:+.2f}:{'ok' if ok else '--'}")
    print(f"z={z} yaw={yaw}", " ".join(row))
