from arm import *
r = Robot()
def qmul(a, b):
    x1,y1,z1,w1 = a; x2,y2,z2,w2 = b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
Q = topdown_quat(0.0)
target = [0.09, -0.188, 0.55]
# Option A: ik_link_name = panda_hand
import moveit_msgs.srv
req = GetPositionIK.Request(); rr = req.ik_request
rr.group_name = "panda_arm"; rr.ik_link_name = "panda_hand"; rr.pose_stamped.header.frame_id = ""
pos = np.array(target) - TCP*quat_to_R(*Q)[:,2]
rr.pose_stamped.pose.position.x, rr.pose_stamped.pose.position.y, rr.pose_stamped.pose.position.z = pos
rr.pose_stamped.pose.orientation.x, rr.pose_stamped.pose.orientation.y, rr.pose_stamped.pose.orientation.z, rr.pose_stamped.pose.orientation.w = Q
rr.robot_state.joint_state.name = JOINTS; rr.robot_state.joint_state.position = r.arm_q(); rr.timeout.sec = 5
res = r._call(r.ik, req); print("A code", res.error_code.val)
if res.error_code.val == 1:
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position)); sol = [sol[j] for j in JOINTS]
    p, q, _ = r.fk_world(sol); print("A FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
# Option B: compensate by +45deg about z
s = np.sin(np.pi/8); c = np.cos(np.pi/8)
Qc = qmul(Q, (0,0,s,c))
sol, code = r.ik_world(target, Qc, at_tcp=False)
if sol:
    p, q, _ = r.fk_world(sol); print("B FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
Qc2 = qmul(Q, (0,0,-s,c))
sol, code = r.ik_world(target, Qc2, at_tcp=False)
if sol:
    p, q, _ = r.fk_world(sol); print("B2 FK:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
