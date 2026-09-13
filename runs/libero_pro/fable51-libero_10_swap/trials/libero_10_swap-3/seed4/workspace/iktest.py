import sys, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
sys.path.insert(0,'/workspace'); from move import quat_R
M = yaml.safe_load(open("/workspace/machine.yaml")); tj=next(e for e in M["actuators"] if e["kind"]=="joint_trajectory")
rclpy.init(); node=rclpy.create_node("ikt"); js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service()
def ik(tcp, q, seed=None):
    R=quat_R(*q); hand=np.array(tcp)-0.1034*R[:,2]
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"; req.ik_request.pose_stamped.header.frame_id=""
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=[float(v) for v in hand]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]
    s=JointState()
    for j in tj["joints"]: s.name.append(j); s.position.append((seed or cur)[j])
    req.ik_request.robot_state.joint_state=s; req.ik_request.avoid_collisions=False
    req.ik_request.timeout.sec=1
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
    if r.error_code.val==1:
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
        return [round(sol[j],3) for j in tj["joints"]]
    return r.error_code.val
tests=[
 ((-0.210,0.0735,1.03),(1,0,0,0)),
 ((-0.210,0.0735,1.03),(0.7071068,0.7071068,0,0)),
 ((-0.210,0.0735,1.03),(0.7071068,-0.7071068,0,0)),
 ((-0.210,0.0735,1.03),(0,0.7071068,0.7071068,0)),
 ((-0.203,0.0,1.166),(1,0,0,0)),
 ((-0.203,0.0,1.166),(0.7071068,0.7071068,0,0)),
]
for tcp,q in tests: print(tcp,q,'->',ik(tcp,q))
