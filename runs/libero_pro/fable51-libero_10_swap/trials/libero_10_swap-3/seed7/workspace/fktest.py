import rclpy, numpy as np
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("t")
js={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
J=[f"panda_joint{i}" for i in range(1,8)]
d=dict(zip(js["m"].name, js["m"].position))
seed=JointState(); seed.name=J; seed.position=[d[j] for j in J]
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]; req.robot_state.joint_state=seed
fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, ps.pose.position)
# IK test with the current hand pose as given by FK
hp=fut.result().pose_stamped[1].pose
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
for label,off in [("as-is",0.0),("minus base",1)]:
    r=GetPositionIK.Request(); r.ik_request.group_name="panda_arm"; r.ik_request.pose_stamped.header.frame_id=""
    r.ik_request.pose_stamped.pose=hp
    if off:
        import copy; p=copy.deepcopy(hp); p.position.x-= -0.66; p.position.z-=0.912; r.ik_request.pose_stamped.pose=p
    r.ik_request.robot_state.joint_state=seed; r.ik_request.avoid_collisions=False
    f=ik.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=60); res=f.result()
    print(label, "code", res.error_code.val, np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)).get(j,0) for j in J],3) if res.error_code.val==1 else "")
print("current", np.round(seed.position,3))
