hp = A.hand_pose_world()
p, q = hp
A.log("current hand", p, q)
A.log("ik at current hand pose (link=panda_hand):", A.solve_ik(p, q, at_tcp=False))
A.log("ik above soup DOWN:", A.solve_ik([-0.185,-0.133,0.70], DOWN, at_tcp=False))
