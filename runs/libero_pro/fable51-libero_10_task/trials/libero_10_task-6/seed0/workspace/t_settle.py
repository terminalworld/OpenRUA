from robot import *
import time
r = Robot("settle")
q0 = r.q(); print("q now", np.round(q0,3), "tcp", np.round(r.tcp()[0],4))
# small move up 3cm at same orientation, then watch joint state vs wall time
pos, quat = r.fk()
qt = r.ik(pos + [0,0,0.03], quat, at_tcp=False, seed=q0)
t0 = time.time()
code, err = r.move(qt, 1.5)
print(f"result at {time.time()-t0:.2f}s code={code} err={err:.4f}")
for i in range(30):
    r.spin(0.2)
    e = np.abs(r.q()-qt).max()
    print(f"  t={time.time()-t0:5.2f}s err={e:.4f} clock={r.node.get_clock().now().nanoseconds/1e9:.3f}")
    if e < 0.003: break
print("tcp", np.round(r.tcp()[0],4))
