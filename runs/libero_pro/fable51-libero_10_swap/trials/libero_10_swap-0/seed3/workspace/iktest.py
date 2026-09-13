from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
log("hand", p.round(4), "quat", np.round(q,4))
# 1) exact current hand pose, at hand (not tcp)
log("A current hand pose:", r.solve_ik(*p, q, at_tcp=False))
# 2) with tcp flag
log("B current tcp pose:", r.solve_ik(*tcp, q, at_tcp=True))
# 3) link8-style quat (rotate by -45deg about z): q_hand * (0,0,0.3827,0.9239)
import math
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
q8 = qmul(q,(0,0,0.38268343,0.92387953))
log("C link8 quat:", r.solve_ik(*p, q8, at_tcp=False))
# 4) frame_id variants
for fid in ("panda_link0","world"):
    req_frame = fid
    from moveit_msgs.srv import GetPositionIK
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    pb = p - BASE_IN_WORLD if fid=="panda_link0" else p
    ps=req.ik_request.pose_stamped.pose
    ps.position.x,ps.position.y,ps.position.z=map(float,pb)
    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w=map(float,q)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in r.arm_q()]
    req.ik_request.robot_state.joint_state=js
    fut=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result(); log("D frame",fid,"code",res.error_code.val if res else None, dict(zip(res.solution.joint_state.name,np.round(res.solution.joint_state.position,3))) if res and res.error_code.val==1 else "")
