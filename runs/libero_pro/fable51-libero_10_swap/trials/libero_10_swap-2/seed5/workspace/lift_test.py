import numpy as np, rclpy
from ctl import Ctl
c = Ctl('lift')
tcp, quat = c.tcp()
print('tcp', np.round(tcp,4), 'fingers', np.round(c.fingers(),4))
q = c.ik(tcp + [0,0,0.04], quat)
print('ik', q is not None)
if q: c.move_q(q, 3.0)
tcp2, _ = c.tcp(); print('tcp now', np.round(tcp2,4), 'fingers', np.round(c.fingers(),4))
c.snap('agentview', 'agent12.png'); c.snap('robot0_eye_in_hand', 'eih12.png')
rclpy.shutdown()
