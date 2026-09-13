import numpy as np
from ctl import Ctl, quat_R, BASE, TCP
c=Ctl()
q0=np.array(c.arm_q())
# FK raw (no BASE added)
xyz,quat=c.hand_pose(); hand_raw = xyz-BASE
print('FK raw hand', np.round(hand_raw,4), 'quat', np.round(quat,4))
R=quat_R(*quat)
# case A: interpret raw as world -> tcp world
tcpA = hand_raw + TCP*R[:,2]
for label, tcp in [('raw-as-world', tcpA), ('raw-as-base(+BASE)', tcpA+BASE)]:
    sol=c.solve_ik(tcp, quat)
    if sol is None: print(label,'no IK'); continue
    print(label, 'max joint diff from current', np.round(np.abs(np.array(sol)-q0).max(),4))
c.close()
