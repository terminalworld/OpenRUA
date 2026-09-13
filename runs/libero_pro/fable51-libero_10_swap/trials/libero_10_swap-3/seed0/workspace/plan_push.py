from robot import *
from moveit_msgs.srv import GetPositionFK
r = Robot("planpush")
Qd = (0.7071,0.7071,0,0); Qp = (0.5,-0.5,0.5,0.5)
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
q0 = np.array(r.arm_q())
qa = r.ik_tcp((-0.19,0.05,1.25), Qd, q0); print("qa", np.round(qa,3))
seedp = [0.01,0.899,0.318,-2.01,-1.421,1.281,-2.567]
qb = r.ik_tcp((-0.19,0.05,1.10), Qp, seedp); print("qb", np.round(qb,3))
t,qq = r.tcp(qb); print(" qb tcp", np.round(t,3), np.round(qq,3))
for i in range(0,21,2):
    q = np.array(qa) + (np.array(qb)-np.array(qa))*i/20
    P = fkall(q); tcp,_ = r.tcp(q)
    print(i, "minz links", round(P[:,2].min(),3), "tcp", np.round(tcp,3), "link7", np.round(P[4],3), "link6", np.round(P[3],3))
np.save("qa.npy", qa); np.save("qb.npy", qb)
