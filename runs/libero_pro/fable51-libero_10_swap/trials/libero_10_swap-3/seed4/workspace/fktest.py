import rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
M = yaml.safe_load(open("/workspace/machine.yaml")); tj=next(e for e in M["actuators"] if e["kind"]=="joint_trajectory")
rclpy.init(); node=rclpy.create_node("fkt"); js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link8","panda_link0"]
s=JointState()
for j in tj["joints"]: s.name.append(j); s.position.append(cur[j])
req.robot_state.joint_state=s
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
print('err',r.error_code.val)
for n,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(n, ps.header.frame_id, f"{p.x:.4f} {p.y:.4f} {p.z:.4f} | {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}")
print('--- FK of IK solutions')
import numpy as np
from move import quat_R
for sol in [[0.126, 0.131, 0.034, -2.549, -0.01, 2.68, 0.169],[0.23, 0.131, -0.065, -2.549, 0.019, 2.68, -1.423],[0.059, -0.171, -0.058, -2.47, -0.013, 2.299, 0.01]]:
    s=JointState()
    for j,v in zip(tj["joints"],sol): s.name.append(j); s.position.append(float(v))
    req.robot_state.joint_state=s; req.fk_link_names=["panda_hand","panda_leftfinger","panda_rightfinger"]
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
    for n,ps in zip(r.fk_link_names,r.pose_stamped):
        p=ps.pose.position;q=ps.pose.orientation
        print(n, f"{p.x:.4f} {p.y:.4f} {p.z:.4f} | {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}", 'hand y-axis in world', np.round(quat_R(q.x,q.y,q.z,q.w)[:,1],3) if n=='panda_hand' else '')
