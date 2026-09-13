from robot import *
from scene import apply_scene, validator
from moveit_msgs.srv import GetPositionFK
r = Robot("planpush2"); print("scene", apply_scene(r)); valid = validator(r)
Q = np.load("Qpush.npy")
LINKS = ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fkall(q):
    req = GetPositionFK.Request(); req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return np.array([(ps.pose.position.x,ps.pose.position.y,ps.pose.position.z) for ps in fut.result().pose_stamped])
qa = np.array(r.arm_q()); print("qa", np.round(qa,3))
seed = [1.259, 0.91, -0.725, -2.425, -1.63, 2.12, -2.282]
qc = r.ik_tcp((-0.19, 0.0, 1.10), Q, seed, avoid=True); print("qc", np.round(qc,3), valid(qc))
t,qq = r.tcp(qc); print(" qc tcp", np.round(t,3), np.round(qq,3))
bad = False
for i in range(0,21):
    q = qa + (np.array(qc)-qa)*i/20
    P = fkall(q); tcp,_ = r.tcp(q); v = valid(q)
    if i%2==0 or not v[0]: print(i, "valid", v, "minz", round(P[:,2].min(),3), "tcp", np.round(tcp,3), "l7", np.round(P[4],3), "l6", np.round(P[3],3), "l4", np.round(P[1],3))
    bad |= not v[0]
print("sweep ok" if not bad else "SWEEP BAD")
# cartesian path check
seed = list(qc); path=[]
for z in np.arange(1.08, 0.97, -0.02): path.append((-0.19,0.0,z))
path.append((-0.19,0.0,0.972))
for y in np.arange(-0.02,-0.245,-0.02): path.append((-0.19,y,0.972))
j6=[]
for p in path:
    q = r.ik_tcp(p, Q, seed, avoid=True)
    if q is None: print("IK FAIL", p); break
    if np.abs(np.array(q)-np.array(seed)).max()>1.0: print("JUMP", p); break
    seed=q; j6.append(q[5])
    print(np.round(p,3), np.round(q,3), valid(q)[0])
np.save("qc.npy", qc)
