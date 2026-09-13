import numpy as np, sys, json
from ctl import *
r=Robot("s6"); chain=json.load(open("chain.json"))
for z,deg,q in chain[2:]:
    r.move_joints(q,seconds=3); p,_=r.hand_pose(); print("z %.3f tilt %d hand %s"%(z,deg,p.round(4)))
r.snap("robot0_eye_in_hand","/workspace/s6_eih.png"); r.snap("sideview","/workspace/s6_side.png"); r.snap("frontview","/workspace/s6_front.png")
