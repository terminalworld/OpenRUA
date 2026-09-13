import sys, numpy as np, rclpy
from ctl import Ctl, JOINTS
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c = Ctl()
cl = c.node.create_client(GetPositionFK, "/compute_fk"); cl.wait_for_service(10)
links = ["panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
for spec in sys.argv[1:]:
    q = [float(v) for v in spec.split(",")]
    req = GetPositionFK.Request(); req.fk_link_names = links
    s = JointState(); s.name = list(JOINTS); s.position = q; req.robot_state.joint_state = s
    f = cl.call_async(req); rclpy.spin_until_future_complete(c.node, f, timeout_sec=30)
    print(spec)
    for ln, ps in zip(links, f.result().pose_stamped):
        p = ps.pose.position; print(f"   {ln:12s} {p.x:7.3f} {p.y:7.3f} {p.z:7.3f}")
