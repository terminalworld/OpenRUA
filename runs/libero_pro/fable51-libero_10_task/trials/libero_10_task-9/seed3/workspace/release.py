from rb import *
r=Robot('rel')
print('open',r.grip(0.04))
p,R=r.fk(); print('hand',p.round(3))
zh=R[:,2]
# retreat along -zh (up/back) by 12 cm
q=r.ik(p-0.12*zh,R); code,err=r.move(q,3)
if err>0.01: code,err=r.move(q,2)
print('retreat',code,err.round(4),r.fk()[0].round(3),'wrench',r.wrench().round(2))
r.snap('agentview','agent_rel.png'); r.snap('frontview','front_rel.png')
