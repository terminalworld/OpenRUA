import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node("fk")
c=n.create_client(GetPositionFK,"/compute_fk"); c.wait_for_service(10)
r=GetPositionFK.Request(); r.fk_link_names=["panda_hand","panda_link8"]
r.robot_state.joint_state=JointState(name=[f"panda_joint{i}" for i in range(1,8)],position=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785])
f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
res=f.result()
print(res.error_code.val)
for nm,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(nm, ps.header.frame_id, round(p.x,3),round(p.y,3),round(p.z,3), [round(v,3) for v in (q.x,q.y,q.z,q.w)])
