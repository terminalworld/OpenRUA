from robot import *
r = Robot("probe")
print("q", np.round(r.arm_q(),4))
pos, quat = r.fk_hand(); print("hand", np.round(pos,4), np.round(quat,4))
tcp, _ = r.tcp(); print("tcp", np.round(tcp,4))
print("fingers", r.fingers())
print("wrench", r.wrench())
# finger positions via TF
from tf2_ros import Buffer, TransformListener
buf = Buffer(); TransformListener(buf, r.node)
for _ in range(20): r.spin(0.1)
for f in ["panda_leftfinger","panda_rightfinger","panda_hand"]:
    t = buf.lookup_transform("world", f, rclpy.time.Time()).transform.translation
    print(f, round(t.x,4), round(t.y,4), round(t.z,4))
# IK test for both candidate orientations
for quat in [(0.7071,-0.7071,0,0),(0.7071,0.7071,0,0)]:
    q = r.ik_tcp([-0.189, 0.022, 1.05], quat)
    print("IK", quat, None if q is None else np.round(q,3))
