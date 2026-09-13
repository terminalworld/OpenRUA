import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("iktest")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service()
def ik(x,y,z,q=(0.9996,0.0,-0.0284,0.0),frame=""):
    x,y,z=float(x),float(y),float(z); q=tuple(float(v) for v in q)
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=frame
    r.pose_stamped.pose.position.x=x; r.pose_stamped.pose.position.y=y; r.pose_stamped.pose.position.z=z
    r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=q
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.ik_link_name="panda_hand"; r.avoid_collisions=False
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
    res=fut.result()
    if res is None: return None, "timeout"
    if res.error_code.val!=1: return None, res.error_code.val
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    return [round(sol[j],3) for j in arm], 1
print("current:", [round(cur[j],3) for j in arm])
print("world coords :", ik(-0.053,0.0,0.7776))
print("base coords  :", ik(0.457,0.0,0.3576))
rclpy.shutdown()
