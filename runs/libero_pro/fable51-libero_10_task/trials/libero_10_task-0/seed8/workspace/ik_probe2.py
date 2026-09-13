import numpy as np, rclpy
from rb import RB, M, ARM, TCP
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot
r = RB("probe2")
p, q, tcp = r.hand_pose()
b = np.array(p) - r.base
def call(variant, seed_joints, with_extras, frame=""):
    req = GetPositionIK.Request()
    req.ik_request.group_name = M["planning"]["group"]
    req.ik_request.pose_stamped.header.frame_id = frame
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float, b)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
    if with_extras:
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
    s = JointState()
    for j, v in seed_joints.items():
        s.name.append(j); s.position.append(float(v))
    req.ik_request.robot_state.joint_state = s
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.n, fut, timeout_sec=60)
    res = fut.result()
    r.log(variant, None if res is None else res.error_code.val,
          None if res is None or res.error_code.val != 1 else np.round(res.solution.joint_state.position, 3))
cur = r.joints()
arm_cur = {j: cur[j] for j in ARM}
home = dict(zip(ARM, [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]))
zeros = dict(zip(ARM, [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0]))
call("cur seed, extras", arm_cur, True)
call("cur seed, no extras", arm_cur, False)
call("home seed, no extras", home, False)
call("zeros seed, no extras", zeros, False)
call("empty seed, no extras", {}, False)
call("cur seed, frame panda_link0", arm_cur, False, "panda_link0")
call("cur seed, frame world", arm_cur, False, "world")
r.close()
