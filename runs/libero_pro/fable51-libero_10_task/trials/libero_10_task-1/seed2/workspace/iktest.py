from rob import *
r = Robot()
tcp, q = r.tcp_world()
tests = [
 ("current", tcp, q),
 ("target q=(1,0,0,0)", [-0.115,-0.153,0.62], (1,0,0,0)),
 ("target cur quat", [-0.115,-0.153,0.62], q),
 ("target yaw45", [-0.115,-0.153,0.62], down_quat(45)),
 ("target yaw-45", [-0.115,-0.153,0.62], down_quat(-45)),
 ("target higher 0.70", [-0.115,-0.153,0.70], (1,0,0,0)),
 ("shift only y", [-0.059,-0.153,0.674], q),
 ("shift only x", [-0.115,0.0,0.674], q),
 ("shift only z", [-0.059,0.0,0.62], q),
]
for name, p, qq in tests:
    try:
        sol = r.ik_world_tcp(p, qq)
        print(name, "OK", [round(v,3) for v in sol])
    except Exception as e:
        print(name, "FAIL", e)
