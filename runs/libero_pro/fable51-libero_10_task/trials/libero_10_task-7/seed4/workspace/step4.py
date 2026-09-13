from rob import *
r = Robot()
print("lift clear")
r.move_tcp([-0.30, -0.139, 0.75], Q_DOWN_X, seconds=4.0)
r.snap("agentview", "/workspace/agent_k3.png")
r.snap("birdview", "/workspace/bird3.png")
print("DONE")
