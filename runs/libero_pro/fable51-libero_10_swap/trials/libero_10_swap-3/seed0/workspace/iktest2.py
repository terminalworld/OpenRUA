from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("iktest2")
Qp = (0.5,-0.5,0.5,0.5)
LIM = np.array(FJT["limits_rad"])
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return {n:(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped)}
rng = np.random.default_rng(0)
sols = []
for i in range(40):
    seed = rng.uniform(LIM[:,0]*0.6, LIM[:,1]*0.6)
    q = r.ik_tcp((-0.19,-0.06,0.955), Qp, seed)
    if q is None: continue
    q = np.array(q)
    if abs(q[6])>2.6 or abs(q[0])>1.2: continue
    fk = fkall(q)
    zmin = min(v[2] for v in fk.values())
    key = tuple(np.round(q,1))
    if any(np.abs(np.array(s[0])-q).max()<0.2 for s in sols): continue
    sols.append((q, zmin, fk))
    print(np.round(q,3), "zmin", round(zmin,3), {k:np.round(v,3).tolist() for k,v in fk.items() if k in ("panda_link4","panda_link6","panda_link7")})
print(len(sols))
