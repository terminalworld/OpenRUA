from rob import *
import rob
rob.BASE=np.zeros(3)
r=Rob("t")
q=r.arm_q()
p,R=r.fk(q)
s=r.ik(p,R)
p2,R2=r.fk(s)
print("pos err", (p2-p).round(4)); print("R target\n",R.round(3)); print("R got\n",R2.round(3))
print("sol", s.round(3))
