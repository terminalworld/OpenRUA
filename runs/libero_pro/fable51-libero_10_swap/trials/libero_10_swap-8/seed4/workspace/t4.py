from robot import *
r=Robot()
print('open', r.gripper(0.04))
j=r.joints(); print('fingers', j['panda_finger_joint1'], j['panda_finger_joint2'])
q=r.arm_q()
for link in ['panda_leftfinger','panda_rightfinger','panda_hand']:
    p,R=r.fk(q,link); print(link,p.round(4))
