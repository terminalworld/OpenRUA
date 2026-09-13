import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
BASE = np.array([0.0, 0.0, 0.0])
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("ikt")
got={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
js=dict(zip(got["m"].name,got["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
def ik(pw, q, timeout=5.0):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"; req.ik_request.ik_link_name="panda_hand"
    req.ik_request.pose_stamped.header.frame_id=""
    pp=req.ik_request.pose_stamped.pose
    pb=np.array(pw)-BASE
    pp.position.x,pp.position.y,pp.position.z=map(float,pb)
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
    seed=JointState(); seed.name=JOINTS; seed.position=[js[j] for j in JOINTS]
    req.ik_request.robot_state.joint_state=seed
    req.ik_request.timeout.sec=int(timeout); req.ik_request.timeout.nanosec=int((timeout%1)*1e9)
    req.ik_request.avoid_collisions=False
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    r=f.result()
    if r is None: return None,None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return r.error_code.val, [round(sol.get(j,float('nan')),3) for j in JOINTS]
tests=[
 ((0.06,0.071,1.3034),(0.7071068,0.7071068,0,0)),
 ((0.06,0.071,1.3034),(1,0,0,0)),
 ((0.06,0.071,1.3034),(0.9238795,0.3826834,0,0)),
 ((0.06,0.071,1.3034),(0,1,0,0)),
 ((-0.203,0,1.27),(1,0,-0.028,0)),
 ((-0.1,0.0,1.25),(0.7071068,0.7071068,0,0)),
 ((0.0,0.07,1.25),(0.7071068,0.7071068,0,0)),
 ((0.06,0.071,1.25),(0.7071068,0.7071068,0,0)),
]
for pw,q in tests:
    print(pw,q,"->",ik(pw,q))
rclpy.shutdown()
