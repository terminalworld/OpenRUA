import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
end = time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
print(buf.all_frames_as_string())
for f in ["panda_link0","panda_hand","agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, f"{tr.x:.4f} {tr.y:.4f} {tr.z:.4f}  q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
    except Exception as e:
        print(f, "ERR", e)
rclpy.shutdown()
