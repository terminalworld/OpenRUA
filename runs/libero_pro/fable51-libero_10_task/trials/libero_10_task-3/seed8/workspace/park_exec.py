from robot import *
r=Robot()
tcp=np.array([-0.15,-0.32,1.25])
q=best_ik(r,tcp,DOWN,n_rand=6); qc=r.arm_q()
for t in (0.5,1.0):
    qq=qc+(q-qc)*t; r.move_q(qq,max(np.abs(qq-r.arm_q()).max()/0.1,1))
print('q',np.round(r.arm_q(),3),'tcp',np.round(r.tcp()[0],3))
