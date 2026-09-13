import sys, math, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("iktry")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
def ik(label,pos,q,frame="",link="",avoid=False):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=frame; r.ik_link_name=link
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=[float(v) for v in pos]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.timeout.sec=1; r.avoid_collisions=avoid
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
    res=fut.result()
    if res is None: print(label,"timeout"); return
    if res.error_code.val!=1: print(f"{label}: FAIL {res.error_code.val}"); return
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(f"{label}: OK", ",".join(f"{sol[j]:.4f}" for j in arm))
qh=(0.9996,0,-0.0284,0)
# link8 quat = qh * Rz(45)
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
q8=qmul(qh,(0,0,0.38268,0.92388)); print("q8",q8)
ik("base,hand q",(0.457,0,0.3576),qh)
ik("base,link8 q",(0.457,0,0.3576),q8)
ik("world,hand q",(-0.053,0,0.7776),qh)
ik("world,link8 q",(-0.053,0,0.7776),q8)
ik("world frame_id,link8 q",(-0.053,0,0.7776),q8,frame="world")
ik("base frame_id,link8 q",(0.457,0,0.3576),q8,frame="panda_link0")
ik("base,link8 q, link=panda_hand",(0.457,0,0.3576),qh,link="panda_hand")
