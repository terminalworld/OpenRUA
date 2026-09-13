from ctl import *
c=Ctl()
print("q",np.round(c.q(),3)); p,o=c.tcp_pose(); print("tcp",np.round(p,4),np.round(o,4))
for tgt,yaw in [((-0.078,-0.189,0.65),0.0),((-0.078,-0.189,0.523),0.0),((-0.006,0.307,0.65),0.0),((-0.03,0.155,0.52),0.0)]:
    qs=c.solve_ik(tgt,topdown_quat(yaw))
    print(tgt, None if qs is None else np.round(qs,3))
c.close()
