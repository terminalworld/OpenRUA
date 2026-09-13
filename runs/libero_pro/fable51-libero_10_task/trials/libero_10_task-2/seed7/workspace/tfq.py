#!/usr/bin/env python3
"""Print transform parent->child as x y z qx qy qz qw."""
import sys, rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfq')
buf=Buffer(); TransformListener(buf,n)
pairs = [tuple(a.split(':')) for a in sys.argv[1:]] or [('world','panda_link0'),('world','panda_hand')]
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.1)
    if all(buf.can_transform(p,c,rclpy.time.Time()) for p,c in pairs): break
for p,c in pairs:
    t=buf.lookup_transform(p,c,rclpy.time.Time())
    tr,q=t.transform.translation,t.transform.rotation
    print(f"{p}->{c}: {tr.x:.4f} {tr.y:.4f} {tr.z:.4f}  q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
