from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.008, *QX)
for i in range(4):
    if r.move(j, 2.0) == 0: break
print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
print("fingers", r.fingers())
