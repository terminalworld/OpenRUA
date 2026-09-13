from robot import *
from scene import validator
from moveit_msgs.srv import GetPositionFK
r = Robot("pushdry2")
valid = validator(r)
th = np.deg2rad(45)
d = np.array([np.sin(th), -np.cos(th), 0.0]); hx = np.array([0,0,1.0]); hy = np.cross(d, hx)
R = np.c_[hx, hy, d]
def R2q(R):
    w = np.sqrt(max(0,1+R[0,0]+R[1,1]+R[2,2]))/2
    x = np.sqrt(max(0,1+R[0,0]-R[1,1]-R[2,2]))/2; y = np.sqrt(max(0,1-R[0,0]+R[1,1]-R[2,2]))/2; z = np.sqrt(max(0,1-R[0,0]-R[1,1]+R[2,2]))/2
    x = np.copysign(x, R[2,1]-R[1,2]); y = np.copysign(y, R[0,2]-R[2,0]); z = np.copysign(z, R[1,0]-R[0,1])
    return np.array([x,y,z,w])
Q = R2q(R); print("Q", np.round(Q,4)); print(np.round(quat_to_R(*Q),3)); print(np.round(R,3))
np.save("Qpush.npy", Q)
LIM = np.array(FJT["limits_rad"])
LINKS = ["panda_link4","panda_link6","panda_link7"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
rng = np.random.default_rng(2)
sols=[]
for i in range(80):
    seed = rng.uniform(LIM[:,0]*0.7, LIM[:,1]*0.7)
    q = r.ik_tcp((-0.19, 0.0, 0.972), Q, seed, avoid=True)
    if q is None: continue
    q=np.array(q)
    if any(np.abs(s-q).max()<0.3 for s in sols): continue
    sols.append(q); P=fkall(q)
    print(np.round(q,3), "j6", round(q[5],2), "l4", np.round(P[0],3), "l6", np.round(P[1],3), "l7", np.round(P[2],3))
