import numpy as np, rclpy, sys
sys.path.insert(0,"/workspace")
from arm import Arm, JOINTS, M
from moveit_msgs.srv import GetPositionIK
a = Arm()
q0 = a.q()
def ik(pos, quat):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = ""
    req.ik_request.ik_link_name = "panda_hand"
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, pos)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = list(map(float, q0))
    req.ik_request.timeout.sec = 2
    fut = a.ik.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=60)
    r = fut.result()
    if r.error_code.val != 1: return r.error_code.val, None
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return 1, np.array([d[j] for j in JOINTS])
quat = [0.9995966352021468, 0, -0.028400121347386478, 0]
print("world-frame pose:", ik([-0.052985648078355034, 0, 0.7776238083193782], quat), "q0", q0.round(3))
print("base-frame pose:", ik([-0.052985648078355034+0.51, 0, 0.7776238083193782-0.42], quat))
