from rob import *
from scene import box
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningScene
r = Robot()
cli = r.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=20)
req = ApplyPlanningScene.Request(); req.scene.is_diff = True
b = box("basket", (0.027, 0.447, 0.525), (0.17, 0.18, 0.20))
t = math.radians(60); b.pose.orientation.z = math.sin(t/2); b.pose.orientation.w = math.cos(t/2)
req.scene.world.collision_objects = [b]
fut = cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30); log("basket CO", fut.result().success)
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=20)
def valid(q):
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(ARM); rq.robot_state.joint_state.position = [float(v) for v in q]
    f = sv.call_async(rq); rclpy.spin_until_future_complete(r.node, f, timeout_sec=30)
    return f.result().valid, [(c.contact_body_1, c.contact_body_2) for c in f.result().contacts]
CX, CY, ang = 0.230, 0.342, math.radians(31.0)
c = np.array([math.cos(ang), math.sin(ang), 0.0]); f = np.array([-math.sin(ang), math.cos(ang), 0.0])
for th in (30, 40):
    tt = math.radians(th); a = math.sin(tt) * c + np.array([0, 0, -math.cos(tt)])
    q = quat_from_axes(a, -f)
    seed = [0.369, 1.114, 0.087, -1.314, -0.009, 2.945, 0.64]
    g = r.solve_ik(CX, CY, 0.45, q, seed=seed)
    pre = np.array([CX, CY, 0.45]) - 0.12 * a
    p = r.solve_ik(*pre, q, seed=g) if g else None
    log(f"tilt {th}: grasp {None if g is None else np.round(g,3)} valid {valid(g) if g else None}")
    log(f"tilt {th}: pre   {None if p is None else np.round(p,3)} valid {valid(p) if p else None}")
