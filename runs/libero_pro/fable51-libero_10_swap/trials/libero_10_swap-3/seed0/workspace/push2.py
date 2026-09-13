import sys
from robot import *
r = Robot("push2")
Q = np.load("Qpush.npy"); qc = list(np.load("qc.npy"))
def rep(tag, code, err):
    t = r.tcp()[0]; w = r.wrench()[0]
    print(tag, "code", code, "err", round(err,4), "tcp", np.round(t,4), "q", np.round(r.arm_q(),3), "wrench", np.round(w,2), flush=True)
    return t, w
for i in range(4):
    code, err = r.move_joints([qc], 12.0 if i==0 else 4.0)
    rep(f"reconf{i}", code, err)
    if err < 0.01: break
else:
    print("reconf did not converge"); sys.exit(1)
rep("down", *r.move_tcp_line([-0.19, 0.0, 0.972], Q, 3.0, avoid=True))
t, f0 = rep("pre", 0, 0.0)
y = 0.0
while y > -0.24:
    y = max(y - 0.02, -0.24)
    try:
        code, err = r.move_tcp_line([-0.19, y, 0.972], Q, 1.0)
    except RuntimeError as e:
        print("stop:", e); break
    t, w = rep(f"push y={y:.2f}", code, err)
    df = w - f0
    print("   dF", np.round(df,2), flush=True)
    if np.linalg.norm(df) > 25 or abs(t[1]-y) > 0.01:
        print("   contact/hard stop -> done pushing", flush=True); break
