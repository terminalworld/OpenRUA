from ctl import *
c=Ctl()
for yaw in (0.0, np.pi/2):
    qs=c.solve_ik((-0.078,-0.187,0.65),topdown_quat(yaw))
    p,o=c.fk_pose(qs); R=quat_R(*o)
    print("yaw",yaw,"q",np.round(qs,3),"hand quat",np.round(o,3),"finger axis (hand y) in world",np.round(R[:,1],3),"hand z",np.round(R[:,2],3))
c.close()
