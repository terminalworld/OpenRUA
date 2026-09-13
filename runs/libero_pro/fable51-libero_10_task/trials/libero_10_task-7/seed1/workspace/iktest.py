from arm import *
a=Arm()
for label,pos in [("base",[0.457,0,0.3576]),("world",[-0.053,0,0.7776])]:
    for q in [(1.0,0,0,0),(0.9996,0,-0.0284,0)]:
        try: print(label,q,np.round(a.solve_ik_base(pos,q,at_tcp=False),4))
        except Exception as e: print(label,q,e)
