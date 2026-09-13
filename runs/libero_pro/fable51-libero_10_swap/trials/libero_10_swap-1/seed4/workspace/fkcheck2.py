from arm import *
a = Arm()
tcp, q = a.tcp_world()
print("TCP world", tcp.round(3), "hand quat", [round(v,3) for v in q])
