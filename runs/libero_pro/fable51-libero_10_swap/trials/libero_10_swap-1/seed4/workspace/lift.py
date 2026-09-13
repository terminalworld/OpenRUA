from arm import Arm
a = Arm()
tcp, _ = a.tcp_world(); print("tcp", tcp.round(3))
a.move((tcp[0], tcp[1], 0.60), seconds=2.5)
a.gripper(0.04)
