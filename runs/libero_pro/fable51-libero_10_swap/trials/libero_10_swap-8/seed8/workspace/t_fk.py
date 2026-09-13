from rob import *
r=Rob("t")
q=r.arm_q(); print("q",q.round(3))
p,R=r.fk(q); print("hand world",p.round(4)); print(R.round(3))
pt,_=r.tcp(q); print("tcp world",pt.round(4))
print("fingers", r.fingers())
