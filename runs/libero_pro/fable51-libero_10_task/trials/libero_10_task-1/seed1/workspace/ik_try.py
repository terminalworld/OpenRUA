"""Try IK for a pose with several yaw variants; print solutions. Usage: ik_try.py x y z [yaw_deg...]"""
import sys, math, numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
x,y,z=map(float,sys.argv[1:4]); yaws=[float(a) for a in sys.argv[4:]] or [0]
rclpy.init(); node=rclpy.create_node("iktry")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service(10)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
for yaw in yaws:
    # top-down: q = (1,0,0,0) rotated about world z by yaw
    h=math.radians(yaw)/2
    # q_z(yaw) * (1,0,0,0): (w,x,y,z) = (cos h,0,0,sin h)*(0,1,0,0) = (0, cos h, sin h, 0)
    qx,qy,qz,qw=math.cos(h),math.sin(h),0.0,0.0
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=""
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=x,y,z
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=qx,qy,qz,qw
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.timeout.sec=2; r.avoid_collisions=False
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=60)
    res=fut.result()
    if res is None: print(yaw,"timeout"); continue
    if res.error_code.val!=1: print(f"yaw {yaw}: FAIL {res.error_code.val}"); continue
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(f"yaw {yaw}: OK", ",".join(f"{sol[j]:.4f}" for j in arm))
