import numpy as np, rclpy
from ctl import Ctl
from cloud import quat_R
c = Ctl('place2')
tcp, quat = c.tcp(); a = quat_R(*quat)[:,2]
print('tcp', np.round(tcp,4), 'a', np.round(a,3))
def go(p, secs=4.0):
    q = c.ik(p, quat, seed=c.arm_q())
    if q is None: raise SystemExit('IK failed')
    code, err = c.move_q(q, secs)
    if code != 0 or err > 0.02: code, err = c.move_q(q, secs)
    t,_ = c.tcp(); print('  tcp', np.round(t,4), 'fingers', np.round(c.fingers(),4), 'F', np.round(c.wrench()[0],1))
go([-0.05, 0.204, 1.03], 3.0)
go([-0.05, 0.204, 0.995], 3.0)
c.gripper(0.08)
c.spin(1.0)
go(np.array([-0.05, 0.204, 0.995]) - 0.09*a, 4.0)   # retreat along -a
print('fingers after retreat', np.round(c.fingers(),4))
go(np.array([-0.05, 0.204, 1.12]) - 0.12*a, 4.0)     # lift away
c.snap('agentview', 'agent14.png'); c.snap('birdview', 'bird14.png'); c.snap('frontview','front14.png')
rclpy.shutdown()
