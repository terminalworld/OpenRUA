import numpy as np, sys
from ctl import *
from geometry_msgs.msg import WrenchStamped
r=Robot("g6")
w={}
r.node.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",lambda m: w.__setitem__("m",m),1)
def wrench():
    w.clear()
    while "m" not in w: rclpy.spin_once(r.node,timeout_sec=0.2)
    f=w["m"].wrench.force; t=w["m"].wrench.torque
    return np.array([f.x,f.y,f.z]).round(2), np.array([t.x,t.y,t.z]).round(2)
q0=topdown_quat(0.0)
xh,yh=-0.468,-0.1276
print("wrench before",wrench())
for z in (1.30,1.26,1.2445,1.239):
    r.move_pose([xh,yh,z],q0,seconds=3)
    print("  z",z,"fingers",np.round(r.finger(),4),"wrench",wrench())
c,d,P,T=r.snap("sideview","/workspace/g6_side.png")
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
sel=Q[(np.hypot(Q[:,0]+0.444,Q[:,1]+0.090)<0.12)&(Q[:,2]>1.02)&(Q[:,2]<1.2)]
for lo in np.arange(1.04,1.2,0.02):
    s=sel[(sel[:,2]>=lo)&(sel[:,2]<lo+0.02)]
    if len(s)>3: print("  z %.2f n=%4d x %.3f..%.3f  y %.3f..%.3f"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
