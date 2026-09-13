import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfd"); b=Buffer(); TransformListener(b,n)
end=time.time()+5
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.1)
print(b.all_frames_as_yaml())
for f in ["agentview_optical_frame","birdview_optical_frame","frontview_optical_frame","sideview_optical_frame","panda_hand","panda_link0"]:
    try:
        t=b.lookup_transform("world",f,rclpy.time.Time()).transform
        print(f, [round(t.translation.x,3),round(t.translation.y,3),round(t.translation.z,3)], [round(t.rotation.x,3),round(t.rotation.y,3),round(t.rotation.z,3),round(t.rotation.w,3)])
    except Exception as e: print(f, "ERR", e)
