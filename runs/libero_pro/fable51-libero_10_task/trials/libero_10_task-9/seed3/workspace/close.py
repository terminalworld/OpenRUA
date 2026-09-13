from rb import *
r=Robot('close')
print('grip',r.grip(0.0))
r.spin(0.5)
print('fingers',r.fingers(),'wrench',r.wrench().round(2))
# lift 4 cm straight up keeping orientation
p,R=r.fk()
q=r.ik(p+np.array([0,0,0.04]),R)
print('lift',r.move(q,2.5), r.fk()[0].round(3))
print('fingers after lift',r.fingers(),'wrench',r.wrench().round(2))
r.snap('agentview','agent_lift.png'); r.snap('frontview','front_lift.png')
