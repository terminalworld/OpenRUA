import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfd'); b=Buffer(); TransformListener(b,n)
for _ in range(30): rclpy.spin_once(n, timeout_sec=0.2)
print(b.all_frames_as_string())
for f in ['panda_link0','agentview_optical_frame','birdview_optical_frame','frontview_optical_frame','sideview_optical_frame','robot0_eye_in_hand_optical_frame','robot0_robotview_optical_frame','panda_hand']:
    try:
        t=b.lookup_transform('world',f,rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f, [round(v,4) for v in (tr.x,tr.y,tr.z)], [round(v,4) for v in (q.x,q.y,q.z,q.w)])
    except Exception as e: print(f,'ERR',e)
