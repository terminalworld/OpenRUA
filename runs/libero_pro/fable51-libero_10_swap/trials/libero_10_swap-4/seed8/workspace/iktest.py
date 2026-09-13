import sys; sys.argv=['x','fk']
import numpy as np, rclpy, time
exec(open('arm.py').read().split("def main")[0])
arm=Arm()
from moveit_msgs.srv import GetPositionIK
for label,p,ac in [('world_noac',(-0.053,0,0.7776),False),('world_ac',(-0.053,0,0.7776),True)]:
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=""
    pp=req.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=map(float,p)
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=1.0,0.0,-0.028,0.0
    req.ik_request.robot_state.joint_state=arm.seed(); req.ik_request.timeout.sec=5
    req.ik_request.avoid_collisions=ac
    t=time.time(); fut=arm.ik.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=40)
    r=fut.result(); print(label, time.time()-t, r and r.error_code.val)
    if r and r.error_code.val==1:
        q=[dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))[j] for j in ARM]
        print(np.round(q,3).tolist()); pos,quat,tcp=arm.fk(q); print('fk of sol', pos.round(4).tolist(), np.round(quat,3).tolist())
