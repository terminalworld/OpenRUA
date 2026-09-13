import numpy as np, rclpy
from ctl import Ctl
c = Ctl('place')
tcp, quat = c.tcp(); print('start tcp', np.round(tcp,4), 'fingers', np.round(c.fingers(),4))
def go(p, secs=4.0):
    q = c.ik(p, quat, seed=c.arm_q())
    if q is None: raise SystemExit('IK failed for %s' % p)
    code, err = c.move_q(q, secs)
    if code != 0 or err > 0.02:
        code, err = c.move_q(q, secs)
    t,_ = c.tcp(); print('  tcp', np.round(t,4), 'fingers', np.round(c.fingers(),4))
    return q
go([tcp[0], tcp[1], 1.10], 4.0)                 # lift clear of pan handle
go([-0.045, -0.03, 1.10], 4.0)                  # midway
go([-0.05, 0.204, 1.10], 4.0)                   # over burner
c.snap('agentview', 'agent13.png')
rclpy.shutdown()
