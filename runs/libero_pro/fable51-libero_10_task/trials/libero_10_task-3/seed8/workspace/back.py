from robot import *
r=Robot()
q0=np.array([0,-0.161,0,-2.445,0,2.227,0.785]); qc=r.arm_q()
for t in (1/3,2/3,1.0):
    q=qc+(q0-qc)*t
    r.move_q(q,3.0)
print(np.round(r.arm_q(),3))
