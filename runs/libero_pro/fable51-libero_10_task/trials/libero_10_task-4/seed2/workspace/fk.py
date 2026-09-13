import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
JOINTS=[f"panda_joint{i}" for i in range(1,8)]
def get_js(node):
    got={}
    s=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s)
    return dict(zip(got["m"].name,got["m"].position))
def fk(node, cli, q, links=("panda_hand","panda_link8")):
    req=GetPositionFK.Request()
    req.header.frame_id=""
    req.fk_link_names=list(links)
    req.robot_state.joint_state.name=JOINTS
    req.robot_state.joint_state.position=[float(x) for x in q]
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
    r=fut.result()
    out={}
    for n,ps in zip(r.fk_link_names,r.pose_stamped):
        p,o=ps.pose.position,ps.pose.orientation
        out[n]=(np.array([p.x,p.y,p.z]),np.array([o.x,o.y,o.z,o.w]),ps.header.frame_id)
    return out, r.error_code.val
if __name__=="__main__":
    rclpy.init(); node=rclpy.create_node("fk")
    cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service(10)
    js=get_js(node)
    q=[js[j] for j in JOINTS]
    print("q",np.round(q,4), "fingers", js["panda_finger_joint1"], js["panda_finger_joint2"])
    out,code=fk(node,cli,q)
    print("code",code)
    for n,(p,o,f) in out.items(): print(n,"frame",f,"pos",np.round(p,4),"quat",np.round(o,4))
    rclpy.shutdown()
