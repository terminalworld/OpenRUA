from rob import *
r = Robot()
print("current q", np.round(r.arm_q(),3))
print("fk", r.fk_hand())
for name, tcp, tilt in [("potA handle", (-0.204,-0.265,1.03),0), ("potB handle",(-0.072,0.162,1.03),0),
                        ("stove near x=.10", (0.10,0.047,1.06),0), ("stove x=.13",(0.13,0.047,1.06),0),
                        ("stove x=.13 tilt30",(0.13,0.047,1.06),0.5), ("stove x=.18 tilt30",(0.18,0.047,1.06),0.5),
                        ("stove x=.18 tilt45",(0.18,0.047,1.06),0.8), ("stove x=.23 tilt45",(0.23,0.047,1.06),0.8)]:
    q = r.ik_tcp(tcp, grasp_R(0, tilt))
    print(name, tilt, None if q is None else np.round(q,3))
