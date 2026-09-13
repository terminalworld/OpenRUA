from ctl import *
r=Robot("step2")
ok=r.move_pose([-0.48,-0.14,1.30], topdown_quat(0), 4)
color,depth,P,T=r.snap("robot0_eye_in_hand","/workspace/eih1.png")
np.save("/workspace/eih1_P.npy",P)
print("cam pos", T[:3,3].round(4))
