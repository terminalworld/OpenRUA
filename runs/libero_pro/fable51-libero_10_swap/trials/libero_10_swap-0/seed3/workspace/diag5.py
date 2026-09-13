from rob import *
from moveit_msgs.srv import GetStateValidity
r = Robot()
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); assert sv.wait_for_service(timeout_sec=20)
def valid(q):
    req = GetStateValidity.Request(); req.group_name = "panda_arm"
    req.robot_state.joint_state.name = list(ARM); req.robot_state.joint_state.position = [float(v) for v in q]
    fut = sv.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
log("current state valid?", valid(r.arm_q()))
log("blocked target valid?", valid([0.35, 1.103, 0.103, -1.321, -0.039, 2.94, 0.653]))

def ik_col(x, y, z, quat, seed):
    p = np.array([x, y, z]) - TCP * quat_to_R(*quat)[:, 2]
    q8 = qmul(quat, (0.0, 0.0, 0.38268343, 0.92387953))
    req = GetPositionIK.Request(); req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = "world"
    ps = req.ik_request.pose_stamped.pose
    ps.position.x, ps.position.y, ps.position.z = map(float, p)
    ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)
    req.ik_request.robot_state.joint_state.name = list(ARM)
    req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
    req.ik_request.avoid_collisions = True
    req.ik_request.timeout.sec = 3
    fut = r.ik.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1: return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [sol[n] for n in ARM]

CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
lim = np.array(FJT["limits_rad"])
rng = np.random.default_rng(0)
found = []
for th in (20, 30, 40, 50, 60):
    t = math.radians(th); a = math.sin(t) * c + np.array([0, 0, -math.cos(t)])
    for sgn in (-1, 1):
        q = quat_from_axes(a, sgn * f)
        seeds = [r.arm_q()] + [rng.uniform(lim[:, 0], lim[:, 1]) for _ in range(6)]
        for i, sd in enumerate(seeds):
            sol = ik_col(CX, CY, 0.45, q, sd)
            if sol is None: continue
            sol = np.array(sol); margin = np.min(np.minimum(sol - lim[:, 0], lim[:, 1] - sol))
            v = valid(sol)
            log(f"tilt {th} sgn {sgn} seed {i}: margin {margin:.2f} valid {v[0]} sol {sol.round(2)}")
            found.append((margin, th, sgn, sol.round(4).tolist()))
found.sort(reverse=True)
log("FOUND", found[:5])
