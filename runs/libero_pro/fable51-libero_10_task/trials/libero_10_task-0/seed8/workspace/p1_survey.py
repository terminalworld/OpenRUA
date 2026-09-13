import numpy as np
from rb import RB, q_down_yaw

r = RB("p1")
r.log("joints", {k: round(v, 3) for k, v in r.joints().items()})

# park TCP off to the -y side so the overhead camera sees the objects
target = [-0.25, -0.45, 0.75]
for yaw in (0, 45, -45, 90, -90, 135):
    try:
        q = q_down_yaw(yaw)
        r.ik_world(target, q)
        r.log("yaw ok", yaw)
        break
    except RuntimeError as e:
        r.log("yaw", yaw, e)
r.goto(target, q, seconds=5.0)

r.snap("birdview", "/workspace/bird2.png")
pc = r.cloud("birdview")
np.save("/workspace/bird_pc.npy", pc)
r.snap("agentview", "/workspace/av3.png")
r.close()
