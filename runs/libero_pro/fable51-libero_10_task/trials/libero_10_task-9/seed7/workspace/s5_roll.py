"""Step A: roll hand +60deg about world y while moving TCP to a safe pre-insert pose."""
import sys
from cart import *
r = Robot()
Rf = np.array([[-1,0,0],[0,0,1],[0,1,0]], float)
deg = float(sys.argv[1]); tgt = np.array([float(v) for v in sys.argv[2:5]])
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix()
tcp0 = pos + 0.1034 * R0[:, 2]
R1 = Rotation.from_euler('y', deg, degrees=True).as_matrix() @ Rf
print("tcp0", np.round(tcp0,3), "-> ", tgt, "q0", np.round(r.joints(),2))
qs = cart_path(r, tcp0, R0, tgt, R1, 8)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = max(abs(np.array(qs[-1]) - q0))
    secs = max(6.0, 6.0 * abs(qs[-1][6] - q0[6]) + 2.0, tot * 4)
    print("running", secs, "s"); ok = run(qs, r, secs)
    print("ok", ok, "q", np.round(r.joints(),3), "fingers", np.round(r.fingers(),4))
    pos, quat = r.fk_world(); R = Rotation.from_quat(quat).as_matrix()
    print("tcp", np.round(pos + 0.1034*R[:,2],3), "y_h", np.round(R[:,1],2), "z_h", np.round(R[:,2],2))
