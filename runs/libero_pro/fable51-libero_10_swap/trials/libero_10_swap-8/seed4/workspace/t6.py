from pp import *
pp=PP(); r=pp.r
print('ZH',ZH.round(3),'y-axis',R_G[:,1].round(3))
q0=r.arm_q()
tests={'B_high':(-0.042-0.094,0.254,1.092),'B_pre':(-0.042-0.094,0.254,0.972),'B_grasp':(-0.042,0.254,0.938),
'A_pre':(-0.195-0.094,-0.2005,0.972),'A_grasp':(-0.195,-0.2005,0.938),
'B_place_above':(0.23,0.08,1.06),'B_place':(0.23,0.08,0.973),'B_retreat':(0.23-0.094,0.08,1.007),
'A_place_above':(0.17,-0.02,1.06),'A_place':(0.17,-0.02,0.973)}
for k,p in tests.items():
    s=r.ik_tcp(p,R_G,seed=q0,timeout=3)
    print(k, None if s is None else s.round(2))
    if s is not None:
        hp,hR=r.fk(s,'panda_hand'); print('   fk tcp',(hp+TCP*hR[:,2]).round(4),'z-axis',hR[:,2].round(3))
