import numpy as np
from rb import RB, q_down_yaw

r = RB("p3")
r.goto([-0.25, -0.45, 0.75], q_down_yaw(45), seconds=5.0)
r.snap("birdview", "/workspace/bird3.png")
np.save("/workspace/bird_pc3.npy", r.cloud("birdview"))
r.snap("agentview", "/workspace/av6.png")
r.close()
