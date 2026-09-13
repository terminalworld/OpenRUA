"""Find yellow-colored points of the held mug in a camera cloud (uses cam_objs machinery)."""
import sys, numpy as np, subprocess
cam = sys.argv[1]
subprocess.run(["timeout", "60", "python3", "cam_objs.py", cam, "0.9", "0.9", "0.95"], capture_output=True)
P = np.load(f"{cam}_P.npy")
import rclpy, time
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node("hm"); got = {}
n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
while "c" not in got: rclpy.spin_once(n, timeout_sec=0.1)
c = np.frombuffer(got["c"].data, dtype=np.uint8).reshape(P.shape[0], P.shape[1], -1)[..., :3].astype(int)
yellow = (c[...,0] > 150) & (c[...,1] > 130) & (c[...,2] < 110) & (c[...,0] - c[...,2] > 70)
sel = yellow & np.isfinite(P[...,2]) & (P[...,2] > 0.55)
pts = P[sel]
print("yellow pts", len(pts))
if len(pts):
    print("x", np.round([pts[:,0].min(), pts[:,0].max()],3), "y", np.round([pts[:,1].min(), pts[:,1].max()],3), "z", np.round([pts[:,2].min(), pts[:,2].max()],3))
    lo = pts[pts[:,2] < pts[:,2].min() + 0.01]
    print("lowest cluster mean", np.round(lo.mean(0),3), "n", len(lo))
    hi = pts[pts[:,2] > pts[:,2].max() - 0.01]
    print("highest cluster mean", np.round(hi.mean(0),3), "n", len(hi))
