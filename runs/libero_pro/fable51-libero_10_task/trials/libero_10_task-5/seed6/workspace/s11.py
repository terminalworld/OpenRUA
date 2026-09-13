import json, numpy as np, cv2
from ctl import Robot
from wr import Wrench
r = Robot("s11"); w = Wrench(r)
chain = json.load(open("chain2.json"))
print("start", r.hand_pose()[0].round(4), "fingers", r.finger(), "wrench", w.get())
prev = np.array(r.arm_q())
for i,(z,deg,q) in enumerate(chain[1:], 1):
    d = np.abs(np.array(q)-prev).max(); sec = max(1.5, d/0.4)
    ok = r.move_joints(q, seconds=sec)
    p,_ = r.hand_pose(); prev = np.array(r.arm_q())
    print(f"[{i}] z={z} tilt={deg} ok={ok} hand={p.round(4)} wrench={w.get()}", flush=True)
    if i == len(chain)-3:
        r.snap("robot0_eye_in_hand", "/workspace/s11_eih_pre.png")
r.snap("robot0_eye_in_hand", "/workspace/s11_eih.png")
r.snap("sideview", "/workspace/s11_side.png")
