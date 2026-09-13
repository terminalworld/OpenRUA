from rob import *
r = Robot()
for name,q in [("P1",[-0.609,0.191,-0.201,-2.851,2.317,1.789,0.889]),("P3",[-0.871,0.675,0.169,-2.502,2.425,1.794,0.686])]:
    L=r.fk_links(q,("panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"))
    print(name); [print("  ",k,v.round(3)) for k,v in L.items()]
    t,R=r.tcp(q); print("   tcp",t.round(3))
