import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
c=Ctl()
cli=c.node.create_client(GetPositionFK,"/compute_fk"); print(cli.wait_for_service(10))
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand"]
d=c.joints(); js=JointState()
for j in ARM: js.name.append(j); js.position.append(d[j])
req.robot_state.joint_state=js
for fid in ["", "world", "panda_link0"]:
    req.header.frame_id=fid
    f=cli.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=30)
    r=f.result()
    print("frame",repr(fid),"code",r.error_code.val if r else None)
    if r:
        for n,p in zip(r.fk_link_names,r.pose_stamped):
            print(" ",n,p.header.frame_id,[round(v,3) for v in (p.pose.position.x,p.pose.position.y,p.pose.position.z)],[round(v,3) for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)])
