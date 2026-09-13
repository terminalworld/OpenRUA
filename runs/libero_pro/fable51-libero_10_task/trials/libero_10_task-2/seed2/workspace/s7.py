from rob import *
r = R()
Q = (1.0, 0.0, 0.0, 0.0)
for z in (0.98, 0.945):
    j = r.ik(-0.197, 0.201, z, *Q)
    for i in range(3):
        if r.move(j, 2.0) == 0: break
    print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
print("fingers", r.fingers())
