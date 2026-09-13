from rob import *
r = Robot()
Q45 = (0.9238795, 0.3826834, 0.0, 0.0)   # hand yaw +45deg, pointing down
print("lift higher")
r.move_tcp([-0.06, -0.146, 0.78], Q_DOWN_Y)
print("gap:", round(r.finger_gap(),4))
print("over basket, diagonal")
r.move_tcp([0.024, 0.284, 0.78], Q45)
print("gap:", round(r.finger_gap(),4))
r.snap("birdview", "/workspace/bird_k6.png")
r.snap("agentview", "/workspace/agent_k6.png")
print("DONE")
