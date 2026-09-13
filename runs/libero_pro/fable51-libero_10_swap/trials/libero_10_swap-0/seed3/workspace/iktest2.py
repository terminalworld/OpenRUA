from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
def ik(fid, timeout=None, avoid=None, pos=None, quat=None):
    req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    pb = pos if pos is not None else p
    ps=req.ik_request.pose_stamped.pose
    ps.position.x,ps.position.y,ps.position.z=map(float,pb)
    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w=map(float,quat or q)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in r.arm_q()]
    req.ik_request.robot_state.joint_state=js
    if timeout is not None: req.ik_request.timeout.sec=timeout
    if avoid is not None: req.ik_request.avoid_collisions=avoid
    fut=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result(); 
    return res.error_code.val, (np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))[n] for n in ARM],3) if res.error_code.val==1 else None)
log("empty frame, world pos:", ik(""))
log("empty frame, base pos:", ik("", pos=p-BASE_IN_WORLD))
log("empty frame, timeout=2:", ik("", timeout=2))
log("world, timeout=2:", ik("world", timeout=2))
log("world, avoid=False:", ik("world", avoid=False))
# check link8 hypothesis: give q8 with world frame -> expect joint7 ~ 0.785 (current)
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
log("world, q8:", ik("world", quat=qmul(q,(0,0,0.38268343,0.92387953))))
log("current q:", np.round(r.arm_q(),3))
