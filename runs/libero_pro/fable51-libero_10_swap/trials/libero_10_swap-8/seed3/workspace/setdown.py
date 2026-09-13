"""Lower the held pot straight down to TCP z given, open, back off, rise."""
import sys
from rob import *
z=float(sys.argv[1]); yaw=float(sys.argv[2]) if len(sys.argv)>2 else 0.0
pitch=float(sys.argv[3]) if len(sys.argv)>3 else 10.0
r=Robot(); R=side_grasp_R(yaw,pitch)
t,_=r.tcp(); print("TCP",t.round(3),"gap",round(r.finger_gap(),4))
tgt=np.array([t[0],t[1],z])
r.move_converged(cart_line(r,R,t,tgt,0.03,r.arm_q()),3.0)
print("TCP",r.tcp()[0].round(3),"gap",round(r.finger_gap(),4))
r.gripper(GRIP["open_m"])
ap=R[:,2].copy(); ap[2]=0; ap/=np.linalg.norm(ap)
t,_=r.tcp()
r.move_converged(cart_line(r,R,t,t-0.07*ap,0.03,r.arm_q()),2.5)
t,_=r.tcp()
r.move_converged(cart_line(r,R,t,t+np.array([0,0,0.08]),0.03,r.arm_q()),2.5)
print("TCP",r.tcp()[0].round(3))
