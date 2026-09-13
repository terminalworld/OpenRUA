from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("iktest3")
LIM = np.array(FJT["limits_rad"])
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
rng = np.random.default_rng(1)
for name, Q in [("mount-up",(0.5,-0.5,0.5,0.5)),("mount-down",(0.5,0.5,-0.5,0.5))]:
    print("==", name, np.round(quat_to_R(*Q),2).tolist())
    sols=[]
    for i in range(60):
        seed = rng.uniform(LIM[:,0]*0.7, LIM[:,1]*0.7)
        q = r.ik_tcp((-0.19,-0.06,0.965), Q, seed, avoid=True)
        if q is None: continue
        q=np.array(q)
        if any(np.abs(s-q).max()<0.3 for s in sols): continue
        sols.append(q); P=fkall(q)
        print(np.round(q,3), "minz", round(P[:,2].min(),3), "l7", np.round(P[4],3), "l6", np.round(P[3],3), "l4", np.round(P[1],3))
