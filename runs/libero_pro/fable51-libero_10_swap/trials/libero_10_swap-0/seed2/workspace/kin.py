import sys, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
ARM=[f"panda_joint{i}" for i in range(1,8)]
rclpy.init(); node=rclpy.create_node("kin")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
seed=JointState()
for n,p in zip(js["m"].name, js["m"].position):
    if n in ARM: seed.name.append(n); seed.position.append(p)
print("seed", dict(zip(seed.name,[round(p,3) for p in seed.position])))
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
r=GetPositionFK.Request(); r.header.frame_id=""; r.fk_link_names=["panda_hand"]; r.robot_state.joint_state=seed
f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
res=f.result(); print("FK err", res.error_code.val)
for ps in res.pose_stamped:
    p=ps.pose.position; o=ps.pose.orientation
    print("FK frame", ps.header.frame_id, "pos", round(p.x,4),round(p.y,4),round(p.z,4), "q", round(o.x,4),round(o.y,4),round(o.z,4),round(o.w,4))
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
def try_ik(x,y,z,qx,qy,qz,qw,frame=""):
    q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
    q.ik_request.pose_stamped.header.frame_id=frame
    pp=q.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=x,y,z
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=qx,qy,qz,qw
    q.ik_request.robot_state.joint_state=seed
    q.ik_request.timeout.sec=1
    f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    r=f.result()
    if r is None: print("  no answer"); return None
    if r.error_code.val!=1: print("  IK err", r.error_code.val); return None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    print("  sol", [round(sol[j],4) for j in ARM]); return [sol[j] for j in ARM]
for a in sys.argv[1:]:
    v=[float(t) for t in a.split(",")]
    print("IK", v); try_ik(*v)
