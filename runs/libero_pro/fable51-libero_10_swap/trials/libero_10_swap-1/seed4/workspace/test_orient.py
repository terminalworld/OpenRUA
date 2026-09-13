from arm import Arm
a = Arm()
a.move((-0.001, -0.274, 0.56), yaw=0.0, seconds=3)
tcp, q = a.tcp_world(); print("hand quat", [round(v,3) for v in q])
