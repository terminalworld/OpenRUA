from rob import *
r = Robot()
for l in ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand","panda_leftfinger","panda_rightfinger"]:
    for _ in range(50):
        rclpy.spin_once(r.node, timeout_sec=0.1)
        if r.tfbuf.can_transform("world", l, rclpy.time.Time()): break
    t = r.tfbuf.lookup_transform("world", l, rclpy.time.Time()).transform.translation
    log(l, round(t.x,3), round(t.y,3), round(t.z,3))
log("q", np.round(r.arm_q(),3))
