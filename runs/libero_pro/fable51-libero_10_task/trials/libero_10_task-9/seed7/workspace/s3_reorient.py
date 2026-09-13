from cart import *
r = Robot()
Rg = np.array([[1,0,0],[0,-1,0],[0,0,-1]], float)
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
tcp0 = current_tcp(r, Rg); print("tcp now", np.round(tcp0,3))
# (a) straight up
qs = cart_path(r, tcp0, Rg, [tcp0[0], tcp0[1], 1.32], Rg, 2)
run(qs, r, 3.0)
# (b) reorient in place
tcpA = current_tcp(r, Rg)
qs = cart_path(r, tcpA, Rg, tcpA, Rf, 6)
print("max jump ok?"); run(qs, r, 8.0)
pos, qt = r.fk_world(); print("hand", np.round(pos,3), "quat", np.round(qt,3), "want", np.round(quat_from_R(Rf),3))
print("fingers", r.fingers())
