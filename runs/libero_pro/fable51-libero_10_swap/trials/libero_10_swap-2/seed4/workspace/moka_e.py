import sys
from moka_plan import *
m = Mover("moka_e")
TH = np.radians(12.0); DT = D*np.cos(TH) - np.array([0,0,1.0])*np.sin(TH); Q_TILT = hand_quat(DT, +CC)
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); print("wrench before", np.round(wrench(m),2))
dx,dy,dz = [float(a) for a in sys.argv[1:4]]; quat = Q_TILT if (len(sys.argv)>4 and sys.argv[4]=="tilt") else Q_GRASP
t = p0 + np.array([dx,dy,dz]); move_interp(m, t, quat, n=3, seconds=2.5)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
print("wrench after", np.round(wrench(m),2))
