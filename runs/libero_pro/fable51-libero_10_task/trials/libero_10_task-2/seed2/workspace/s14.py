from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
r.grip(0.04)
for z,secs in ((1.05,3.0),(0.948,2.5)):
    j = r.ik(-0.058, -0.075, z, *QX)
    for i in range(4):
        if r.move(j, secs) == 0: break
    print("tcp", r.tcp()[0].round(4))
r.grip(0.0)
# settle ticks: tiny hold trajectory to let the sim advance
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
