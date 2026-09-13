import rclpy, move, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
mv = move.Mover()
cur = mv.joints()
def ik(pos, quat, link):
    req = GetPositionIK.Request()
    req.ik_request.group_name='panda_arm'; req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=map(float,pos)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    seed=JointState()
    for j in move.JOINTS: seed.name.append(j); seed.position.append(cur[j])
    req.ik_request.robot_state.joint_state=seed
    req.ik_request.timeout=Duration(sec=1)
    fut=mv.ik.call_async(req); rclpy.spin_until_future_complete(mv.node,fut,timeout_sec=60)
    r=fut.result()
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position)) if r.error_code.val==1 else {}
    print(link, pos, r.error_code.val, [round(sol.get(j,0),3) for j in move.JOINTS], flush=True)
hq=(0.9996,0,-0.0284,0)
ik((-0.053,0,0.7776), hq, 'panda_hand')
ik((0.457,0,0.3576), hq, 'panda_hand')
ik((-0.053,0,0.7776), (1,0,0,0), 'panda_hand')
ik((0.457,0,0.3576), (1,0,0,0), 'panda_hand')
ik((-0.097,0.057,0.55+0.1034), (1,0,0,0), 'panda_hand')
ik((0.413,0.057,0.55+0.1034-0.42), (1,0,0,0), 'panda_hand')
