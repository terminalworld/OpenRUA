import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfd"); b=Buffer(); TransformListener(b,n)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
frames = b.all_frames_as_yaml()
print(frames)
for f in ["panda_link0","panda_hand","agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    try:
        t=b.lookup_transform("world",f,rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f}", f"q={q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e: print(f, "ERR", e)
