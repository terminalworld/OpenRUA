from rob import *
r = Robot()
K = np.array([-0.197, -0.139])
print("open"); r.gripper(0.04)
print("descend")
r.move_tcp([K[0], K[1], 0.47], Q_DOWN_X, seconds=4.0)
r.snap("robot0_eye_in_hand", "/workspace/eih_k1.png")
r.snap("agentview", "/workspace/agent_k1.png")
print("DONE")
