from ctl import *
r=Robot("step3")
ok=r.move_pose([-0.122+0.035, 0.044-0.035, 1.25], topdown_quat(np.pi/2), 4)
p,q=r.hand_pose(); print("hand", p.round(4), q.round(4))
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih2.png")
np.save("/workspace/eih2_P.npy",P)
print("cam pos", T[:3,3].round(4))
