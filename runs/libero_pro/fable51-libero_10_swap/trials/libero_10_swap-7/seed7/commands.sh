#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter; echo ---; timeout 20 ros2 node list

# openrua op 2
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 3
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[190:290,190:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_box.png', cv2.resize(im[350:430,100:280],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('frontview.png')
cv2.imwrite('crop_front.png', cv2.resize(im[200:290,220:380],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png')
cv2.imwrite('crop_side.png', cv2.resize(im[400:480,220:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>&1 | grep -A 12 "child_frame_id: panda_link0"; echo; echo "soup can:"; timeout 60 python3 tools/perception/px2world.py agentview 230 240; echo "tomato:"; timeout 60 python3 tools/perception/px2world.py agentview 372 240; echo "box:"; timeout 60 python3 tools/perception/px2world.py agentview 143 392; echo "basket rim front-left:"; timeout 60 python3 tools/perception/px2world.py agentview 470 200; echo "basket inner floor:"; timeout 60 python3 tools/perception/px2world.py agentview 530 240

# openrua op 7
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_bird.png', cv2.resize(im[210:340,240:430],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"; echo "table near box:"; timeout 60 python3 tools/perception/px2world.py agentview 300 420; echo "table between:"; timeout 60 python3 tools/perception/px2world.py birdview 320 330

# openrua op 8
cd /workspace; for p in "287 235" "295 301" "272 312" "385 290" "383 273" "383 310"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helper: FK / IK / trajectory / gripper / servo, one node.

World <-> base conversion uses the fixed world->panda_link0 transform
read from /tf at import time. Poses given to ik/move are WORLD-frame
TCP (fingertip) poses unless at="hand".
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# top-down grasp: hand +Z -> world -Z, fingers open along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world Z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self._base = None
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        t0 = time.time()
        while (self._js is None or self._base is None) and time.time() - t0 < 15:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._base is None:
            self._base = np.array([-0.51, 0.0, 0.42])
            print("WARN: no world->panda_link0 on /tf, using manifest default")
        print("base in world:", self._base)

    def _on_js(self, msg):
        self._js = msg

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "panda_link0" and t.header.frame_id == "world":
                tr = t.transform.translation
                self._base = np.array([tr.x, tr.y, tr.z])

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self, at="tcp"):
        """World-frame position of hand or TCP + quaternion (x,y,z,w)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        seed.name = list(JOINTS)
        seed.position = self.arm_q()
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self._base
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        if at == "tcp":
            pos = pos + TCP * quat_to_R(q)[:, 2]
        return pos, q

    # ---- planning ----
    def solve_ik(self, pos_world, q=Q_DOWN, at="tcp", seed=None):
        pos = np.array(pos_world, dtype=float)
        if at == "tcp":
            pos = pos - TCP * quat_to_R(q)[:, 2]
        pos = pos - self._base
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        s = JointState()
        s.name = list(JOINTS)
        s.position = list(seed) if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- acting ----
    def move_joints(self, targets, seconds=3.0, tol=0.02):
        """targets: list of joint vectors (waypoints) or a single vector."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [targets]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(targets)
        for i, tq in enumerate(targets):
            pt = JointTrajectoryPoint(positions=[float(v) for v in tq])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(targets[-1])).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, pos_world, q=Q_DOWN, seconds=3.0, at="tcp", seed=None):
        sol = self.solve_ik(pos_world, q, at, seed)
        if sol is None:
            print(f"  IK FAILED for {np.round(pos_world, 3)}")
            return False
        self.move_joints(sol, seconds)
        p, _ = self.hand_pose(at)
        print(f"  now at {at} {np.round(p, 4)} (target {np.round(pos_world, 4)})")
        return True

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        g = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={g}")
        return g

    def servo(self, v, n=20):
        """Stream n twist messages with linear velocity v (m/s, base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
OPENRUA_EOF

# openrua op 10
timeout 120 python3 -u -c "
from arm import *
a = Arm()
print('q', np.round(a.arm_q(),3))
print('fingers', a.finger_gap())
p,q = a.hand_pose('hand'); print('hand', np.round(p,4), np.round(q,4))
p,q = a.hand_pose('tcp'); print('tcp', np.round(p,4))
# test IK above soup
for z in (0.62, 0.55):
    s = a.solve_ik([-0.22,-0.143,z]); print('ik soup z',z, s and np.round(s,3))
s = a.solve_ik([0.12,-0.21,0.55]); print('ik box', s and np.round(s,3))
s = a.solve_ik([0.02,0.28,0.70]); print('ik basket', s and np.round(s,3))
"

# openrua op 11
timeout 120 python3 -u -c "
from arm import *
a = Arm()
p,q = a.hand_pose('hand')
import rclpy
from moveit_msgs.srv import GetPositionIK
def raw(pos_base, q, timeout=2, avoid=False):
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'
    req.ik_request.pose_stamped.header.frame_id = ''
    pp = req.ik_request.pose_stamped.pose
    pp.position.x, pp.position.y, pp.position.z = map(float,pos_base)
    pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float,q)
    s = JointState(); s.name=list(JOINTS); s.position=a.arm_q()
    req.ik_request.robot_state.joint_state = s
    req.ik_request.timeout.sec = timeout
    req.ik_request.avoid_collisions = avoid
    fut = a.ik.call_async(req); rclpy.spin_until_future_complete(a.node, fut, timeout_sec=60)
    r = fut.result(); return r.error_code.val, dict(zip(r.solution.joint_state.name, np.round(r.solution.joint_state.position,3)))
print('current pose', raw(p - a._base, q))
print('current pose, q_down', raw(p - a._base, Q_DOWN))
print('above soup hand z0.35', raw([0.29,-0.143,0.35], Q_DOWN))
print('above soup hand z0.35 avoid', raw([0.29,-0.143,0.35], Q_DOWN, avoid=True))
print('above soup hand z0.35 5s', raw([0.29,-0.143,0.35], Q_DOWN, timeout=5))
"

# openrua op 12
timeout 300 python3 -u -c "
from arm import *
a = Arm()
home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
tests = [([0.5,0,0.3],Q_DOWN),([0.4,-0.14,0.3],Q_DOWN),([0.29,-0.143,0.35],Q_DOWN),([0.29,-0.143,0.35],q_down_yaw(0.785)),([0.29,-0.143,0.35],q_down_yaw(-0.785)),([0.3,0,0.5],Q_DOWN),([0.3,0,0.6],Q_DOWN)]
for pos,q in tests:
    for name,seed in (('cur',None),('home',home)):
        s = a.solve_ik(np.array(pos)+a._base, q, at='hand', seed=seed)
        print(pos, np.round(q,3), name, s and np.round(s,3))
"

# openrua op 13
timeout 30 ros2 param list /move_group 2>/dev/null | grep -iE "kinematics|semantic|panda_arm|solver|tip|joint_limits" | head -30; echo ---; timeout 30 ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -E "group|chain|link|joint" | head -40

# openrua op 14
for p in kinematics_solver kinematics_solver_timeout position_only_ik epsilon max_solver_iterations kinematics_solver_search_resolution; do echo -n "$p: "; timeout 20 ros2 param get /move_group robot_description_kinematics.panda_arm.$p; done; echo; timeout 20 ros2 param get /move_group robot_description_planning.joint_limits.panda_joint1.max_position 2>&1; timeout 20 ros2 param get /move_group robot_description_planning.joint_limits.panda_joint4.min_position 2>&1

# openrua op 15
timeout 60 python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)  # link8 / flange
    return T
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
T=fk(q); print('link8 pos', np.round(T[:3,3],4)); print(np.round(T[:3,:3],3))
"


# openrua op 16
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('/workspace/robot.urdf','w').write(got[0].data)
print(len(got[0].data))
"; grep -nE "<joint |<origin|<parent|<child|<limit" /workspace/robot.urdf | grep -vE "visual|collision" | head -80

# openrua op 17
python3 -c "
import re
s=open('/workspace/robot.urdf').read()
for m in re.finditer(r'<joint[^>]*name=\"([^\"]+)\"[^>]*type=\"([^\"]+)\"[^>]*>(.*?)</joint>', s, re.S):
    body=m.group(3)
    o=re.search(r'<origin[^>]*/>',body); p=re.search(r'<parent[^>]*/>',body); c=re.search(r'<child[^>]*/>',body); l=re.search(r'<limit[^>]*/>',body); ax=re.search(r'<axis[^>]*/>',body)
    print(m.group(1),m.group(2)); print('  ',o and o.group(0)); print('  ',p and p.group(0), c and c.group(0)); print('  ',ax and ax.group(0)); print('  ',l and l.group(0))
"

# openrua op 18
python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace("pos = np.array([p.position.x, p.position.y, p.position.z]) + self._base","pos = np.array([p.position.x, p.position.y, p.position.z])  # FK is already world-frame")
s=s.replace("        pos = pos - self._base\n        req = GetPositionIK.Request()","        # IK poses are world-frame on this machine (verified against FK)\n        req = GetPositionIK.Request()")
open('arm.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from arm import *
a = Arm()
p,q = a.hand_pose('hand'); print('hand', np.round(p,4), np.round(q,4))
p,q = a.hand_pose('tcp'); print('tcp', np.round(p,4))
for pos in ([-0.22,-0.143,0.62],[-0.22,-0.143,0.50],[0.12,-0.21,0.60],[0.12,-0.21,0.47],[0.02,0.28,0.72]):
    s = a.solve_ik(pos); print(pos, s and np.round(s,3))
"

# openrua op 19
timeout 120 python3 -u -c "
import numpy as np, rclpy, struct
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from arm import quat_to_R
rclpy.init(); n=rclpy.create_node('seg')
buf=Buffer(); TransformListener(buf,n)
got={}
n.create_subscription(Image,'/birdview/depth/image_raw',lambda m: got.setdefault('d',m),1)
n.create_subscription(CameraInfo,'/birdview/color/camera_info',lambda m: got.setdefault('i',m),1)
while len(got)<2 or not buf.can_transform('world','birdview_optical_frame',rclpy.time.Time()): rclpy.spin_once(n,timeout_sec=0.2)
d=got['d']; D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=buf.lookup_transform('world','birdview_optical_frame',rclpy.time.Time())
q=t.transform.rotation; R=quat_to_R((q.x,q.y,q.z,q.w)); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
v,u=np.mgrid[0:d.height,0:d.width]
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)@R.T+T
np.save('bird_world.npy',P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
def region(name,xr,yr,zmin):
    m=(X>xr[0])&(X<xr[1])&(Y>yr[0])&(Y<yr[1])&(Z>zmin)
    if m.sum()==0: print(name,'none'); return
    print(name,'n',m.sum(),'center',np.round([X[m].mean(),Y[m].mean()],4),'x',np.round([X[m].min(),X[m].max()],3),'y',np.round([Y[m].min(),Y[m].max()],3),'ztop',np.round(Z[m].max(),4))
region('soup',(-0.30,-0.14),(-0.22,-0.07),0.47)
region('box',(0.05,0.20),(-0.28,-0.15),0.44)
region('basket',(-0.15,0.25),(0.15,0.45),0.55)
region('ketchup',(0.0,0.12),(-0.15,-0.05),0.50)
"

# openrua op 20
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can and drop it in the basket."""
from arm import *

SOUP = np.array([-0.2226, -0.1442])
BASKET = np.array([0.0, 0.255])
Z_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.455, 0.72, 0.75

a = Arm()
print("start tcp", np.round(a.hand_pose()[0], 4))

print("== open gripper")
a.gripper(GRIP["open_m"])

print("== hover above soup")
assert a.move_to([*SOUP, Z_HOVER], seconds=4)

print("== descend to grasp height")
assert a.move_to([*SOUP, Z_GRASP], seconds=3)

print("== close gripper")
g = a.gripper(GRIP["closed_m"])
print("finger gap (sum) =", g[0] + abs(g[1]))

print("== lift")
assert a.move_to([*SOUP, Z_CARRY], seconds=3)
print("fingers after lift", a.finger_gap())

print("== move above basket")
assert a.move_to([*BASKET, Z_DROP], seconds=4)
print("fingers above basket", a.finger_gap())

print("== release")
a.gripper(GRIP["open_m"])

print("== retreat up")
assert a.move_to([*BASKET, Z_DROP + 0.05], seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 21
nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 45; cat /workspace/pick_soup.log

# openrua op 22
sleep 30; cat /workspace/pick_soup.log | tail -8; pgrep -f pick_soup.py || echo "finished"

# openrua op 23
sleep 3; pgrep -f pick_soup.py || echo finished; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 24
python3 - <<'EOF'
s=open('arm.py').read()
old_start = s.index("    def move_joints(")
old_end = s.index("    def gripper(")
new = '''    def settle(self, max_reads=60):
        """Poll joint states until the arm stops moving; return final q."""
        prev = np.array(self.arm_q())
        still = 0
        for _ in range(max_reads):
            q = np.array(self.arm_q())
            if np.abs(q - prev).max() < 1e-4:
                still += 1
                if still >= 3:
                    break
            else:
                still = 0
            prev = q
        return prev

    def move_joints(self, targets, seconds=3.0, tol=0.01, retries=3):
        """targets: list of joint vectors (waypoints) or a single vector.
        Sends the goal, waits for the result, waits for the arm to stop,
        and re-sends the final target if it did not converge (-5 on a
        long goal is usually controller lag, see docs/30-action.md)."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [targets]
        final = np.array(targets[-1], dtype=float)
        for attempt in range(retries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            n = len(targets)
            for i, tq in enumerate(targets):
                pt = JointTrajectoryPoint(positions=[float(v) for v in tq])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(pt)
            fut = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            gh = fut.result()
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            q = self.settle()
            err = np.abs(q - final).max()
            print(f"  traj error_code={code} max_joint_err={err:.4f} (attempt {attempt + 1})")
            if err <= tol:
                return code, err
            targets = [final]          # re-send just the final point
            seconds = max(2.0, seconds * 0.6)
        return code, err

    def move_to(self, pos_world, q=Q_DOWN, seconds=3.0, at="tcp", seed=None,
                steps=1, tol_m=0.01):
        """Move the TCP (or hand) to a world pose. steps>1 interpolates a
        straight Cartesian line from the current pose, IK-seeding each
        waypoint with the previous one for joint-space continuity."""
        target = np.array(pos_world, dtype=float)
        sols = []
        if steps > 1:
            start, _ = self.hand_pose(at)
            prev = self.arm_q()
            for i in range(1, steps + 1):
                wp = start + (target - start) * i / steps
                sol = self.solve_ik(wp, q, at, seed=prev)
                if sol is None:
                    print(f"  IK FAILED for waypoint {np.round(wp, 3)}")
                    return False
                sols.append(sol)
                prev = sol
        else:
            sol = self.solve_ik(target, q, at, seed)
            if sol is None:
                print(f"  IK FAILED for {np.round(target, 3)}")
                return False
            sols = [sol]
        self.move_joints(sols, seconds)
        p, _ = self.hand_pose(at)
        d = np.linalg.norm(p - target)
        print(f"  now at {at} {np.round(p, 4)} (target {np.round(target, 4)}, off {d*1000:.1f} mm)")
        return d <= tol_m

'''
s = s[:old_start] + new + s[old_end:]
open('arm.py','w').write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/arm.py').read());print('ok')"

# openrua op 25
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can and drop it in the basket."""
from arm import *

SOUP = np.array([-0.2226, -0.1442])
BASKET = np.array([0.0, 0.255])
Z_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.455, 0.72, 0.75

a = Arm()
print("start tcp", np.round(a.hand_pose()[0], 4))

print("== open gripper")
a.gripper(GRIP["open_m"])

print("== hover above soup")
assert a.move_to([*SOUP, Z_HOVER], seconds=5)

print("== descend to grasp height")
assert a.move_to([*SOUP, Z_GRASP], seconds=5, steps=4)

print("== close gripper")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", gap)
assert gap > 0.02, "closed on air"

print("== lift")
assert a.move_to([*SOUP, Z_CARRY], seconds=4, steps=3)
print("fingers after lift", a.finger_gap())

print("== move above basket")
assert a.move_to([*BASKET, Z_DROP], seconds=5)
print("fingers above basket", a.finger_gap())

print("== release")
a.gripper(GRIP["open_m"])

print("== retreat up")
a.move_to([*BASKET, Z_DROP + 0.05], seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 26
nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 60; cat /workspace/pick_soup.log

# openrua op 27
sleep 60; cat /workspace/pick_soup.log | tail -12

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 120 python3 -u -c "
from arm import *
a=Arm()
q=a.arm_q(); print('cur q', np.round(q,3)); p,_=a.hand_pose(); print('tcp', np.round(p,4))
SOUP=[-0.2226,-0.1442]
prev=q
for z in (0.62,0.58,0.54,0.50,0.455):
    s=a.solve_ik([*SOUP,z],seed=prev); print(z, np.round(s,3), 'jump', np.round(np.abs(np.array(s)-np.array(prev)).max(),3)); prev=s
"

# openrua op 29
timeout 200 python3 -u -c "
from arm import *
a=Arm()
SOUP=[-0.2226,-0.1442]
def q_tilt(th):  # rotate Q_DOWN about world Y by th: fingers point (-sin th, 0, -cos th)
    qy=(0.0,np.sin(th/2),0.0,np.cos(th/2)); return quat_mul(qy,Q_DOWN)
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
for deg in (0,15,25,35,45):
    th=np.radians(deg); q=q_tilt(th)
    print('tilt',deg, 'fingers dir', np.round(quat_to_R(q)[:,2],2))
    prev=ready; ok=True
    for z in (0.62,0.58,0.54,0.50,0.455):
        s=a.solve_ik([*SOUP,z],q=q,seed=prev)
        if s is None: print('   z',z,'IK fail'); ok=False; break
        print('   z',z,np.round(s,2),'jump',np.round(np.abs(np.array(s)-np.array(prev)).max(),2)); prev=s
"

# openrua op 30
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can and drop it in the basket."""
from arm import *

SOUP = np.array([-0.2226, -0.1442])
BASKET = np.array([0.0, 0.255])
Z_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.455, 0.72, 0.75
TILT = np.radians(30)
# rotate Q_DOWN about world Y: fingers point (-sin t, 0, -cos t), i.e.
# down and toward the arm base, so the wrist sits farther out
Q_SOUP = quat_mul((0.0, np.sin(TILT / 2), 0.0, np.cos(TILT / 2)), Q_DOWN)
READY = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

a = Arm()
print("start tcp", np.round(a.hand_pose()[0], 4))

print("== open gripper")
a.gripper(GRIP["open_m"])

print("== hover above soup (tilted)")
assert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=5, seed=READY)

print("== descend to grasp height")
assert a.move_to([*SOUP, Z_GRASP], q=Q_SOUP, seconds=5, steps=4)

print("== close gripper")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", gap)
assert gap > 0.02, "closed on air"

print("== lift")
assert a.move_to([*SOUP, Z_CARRY], q=Q_SOUP, seconds=4, steps=3)
print("fingers after lift", a.finger_gap())

print("== move above basket")
assert a.move_to([*BASKET, Z_DROP], seconds=5)
print("fingers above basket", a.finger_gap())

print("== release")
a.gripper(GRIP["open_m"])

print("== retreat up")
a.move_to([*BASKET, Z_DROP + 0.05], seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 31
nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 90; cat /workspace/pick_soup.log

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview > /dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[120:300,120:340],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 -E "force|torque"; timeout 120 python3 -u -c "
from arm import *
a=Arm()
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_leftfinger','panda_rightfinger','panda_link8']
s=JointState(); s.name=list(JOINTS); s.position=a.arm_q(); req.robot_state.joint_state=s
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
for n,ps in zip(req.fk_link_names,fut.result().pose_stamped):
    p=ps.pose.position; print(n, np.round([p.x,p.y,p.z],4))
print('fingers', a.finger_gap())
"

# openrua op 34
python3 -c "
import cv2
im=cv2.imread('sideview.png'); cv2.imwrite('crop_s.png', cv2.resize(im[250:480,150:500],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 35
mkdir -p "$(dirname /workspace/seg.py)"
cat > /workspace/seg.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects from a camera's depth image, in world coordinates.

Usage: python3 seg.py <camera> name:xmin,xmax,ymin,ymax,zmin [...]
Prints centroid, extents and top height of the points inside each box.
Saves <camera>_world.npy (HxWx3 world points).
"""
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from arm import quat_to_R


def cloud(cam):
    rclpy.init()
    n = rclpy.create_node("seg")
    buf = Buffer()
    TransformListener(buf, n)
    got = {}
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    frame = f"{cam}_optical_frame"
    while len(got) < 2 or not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(n, timeout_sec=0.2)
    d = got["d"]
    D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    k = got["i"].k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R((q.x, q.y, q.z, q.w))
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    v, u = np.mgrid[0:d.height, 0:d.width]
    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D], -1) @ R.T + T
    rclpy.shutdown()
    return P


def region(P, name, xr, yr, zmin, zmax=2.0):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zmin) & (Z < zmax)
    if m.sum() == 0:
        print(name, "none")
        return None
    c = np.array([X[m].mean(), Y[m].mean()])
    print(f"{name}: n={m.sum()} center={np.round(c, 4)} x=[{X[m].min():.3f},{X[m].max():.3f}] "
          f"y=[{Y[m].min():.3f},{Y[m].max():.3f}] ztop={Z[m].max():.4f}")
    return c


if __name__ == "__main__":
    cam = sys.argv[1]
    P = cloud(cam)
    np.save(f"{cam}_world.npy", P)
    for spec in sys.argv[2:]:
        name, box = spec.split(":")
        vals = [float(x) for x in box.split(",")]
        region(P, name, vals[0:2], vals[2:4], vals[4], vals[5] if len(vals) > 5 else 2.0)
OPENRUA_EOF

# openrua op 36
timeout 120 python3 seg.py agentview "soup:-0.32,-0.14,-0.25,-0.05,0.43,0.52" "soup_low:-0.32,-0.14,-0.25,-0.05,0.43,0.47" "hand:-0.25,0.0,-0.25,-0.05,0.50,0.70"; timeout 120 python3 seg.py frontview "soup:-0.32,-0.14,-0.25,-0.05,0.43,0.52"

# openrua op 37
timeout 600 python3 -u -c "
from arm import *
a=Arm()
TILT=np.radians(30); Q_SOUP=quat_mul((0.0,np.sin(TILT/2),0.0,np.cos(TILT/2)),Q_DOWN)
p,_=a.hand_pose()
print('lift'); a.move_to([p[0],p[1],0.65], q=Q_SOUP, seconds=4, steps=2)
print('ready pose'); a.move_joints([0,-0.785,0,-2.356,0,1.571,0.785], seconds=5)
print('tcp', np.round(a.hand_pose()[0],4))
" 2>&1 | tail -8; timeout 120 python3 seg.py birdview "soup:-0.32,-0.10,-0.25,-0.05,0.47" "box:0.05,0.20,-0.28,-0.15,0.44" "ketchup:0.0,0.12,-0.15,-0.05,0.50" "tomato:-0.25,-0.05,0.0,0.15,0.47"

# openrua op 38
timeout 120 python3 seg.py birdview "soup:-0.32,-0.10,-0.25,-0.05,0.47,0.52" "tomato:-0.25,-0.05,0.0,0.15,0.47,0.52"; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null

# openrua op 39
timeout 600 python3 -u -c "
from arm import *
a=Arm()
a.move_to([0.0,0.255,0.80], seconds=5)
" 2>&1 | tail -2; timeout 120 python3 seg.py birdview "soup:-0.32,-0.10,-0.25,-0.05,0.47,0.52" "tomato:-0.25,-0.05,0.0,0.15,0.47,0.52" "box:0.05,0.20,-0.28,-0.15,0.44,0.5"

# openrua op 40
timeout 15 ros2 topic echo /tf --once 2>/dev/null | grep child_frame_id; timeout 15 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once 2>/dev/null | grep -E "frame_id|k:" -A1 | head; timeout 15 ros2 topic echo /agentview/color/camera_info --once 2>/dev/null | grep -E "frame_id"

# openrua op 41
python3 - <<'EOF'
s=open('/workspace/seg.py').read()
s=s.replace('''def cloud(cam):
    rclpy.init()
    n = rclpy.create_node("seg")
    buf = Buffer()
    TransformListener(buf, n)''','''def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init()
    n = node or rclpy.create_node("seg")
    buf = Buffer()
    tl = TransformListener(buf, n)''')
s=s.replace('''    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D], -1) @ R.T + T
    rclpy.shutdown()
    return P''','''    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D], -1) @ R.T + T
    tl.unregister()
    if own:
        rclpy.shutdown()
    return P''')
open('/workspace/seg.py','w').write(s)
EOF
echo ok

# openrua op 42
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can and drop it in the basket."""
import sys

from arm import *
from seg import cloud, region

SOUP = np.array([-0.2131, -0.1331])          # latest birdview estimate
BASKET = np.array([0.0, 0.255])
Z_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.455, 0.72, 0.75
TILT = np.radians(30)
# rotate Q_DOWN about world Y: fingers point (-sin t, 0, -cos t), i.e.
# down and toward the arm base, so the wrist sits farther out
Q_SOUP = quat_mul((0.0, np.sin(TILT / 2), 0.0, np.cos(TILT / 2)), Q_DOWN)
READY = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

a = Arm()
print("start tcp", np.round(a.hand_pose()[0], 4))

print("== open gripper")
a.gripper(GRIP["open_m"])

print("== hover above soup (tilted)")
assert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=5, seed=READY)

print("== re-measure can from eye-in-hand + agentview")
box = (SOUP[0] - 0.08, SOUP[0] + 0.08, SOUP[1] - 0.08, SOUP[1] + 0.08)
c_eih = region(cloud("robot0_eye_in_hand", a.node), "eih", box[0:2], box[2:4], 0.47, 0.53)
c_agv = region(cloud("agentview", a.node), "agv", box[0:2], box[2:4], 0.47, 0.53)
if c_eih is not None and np.linalg.norm(c_eih - SOUP) < 0.03:
    SOUP = c_eih
    print("using eye-in-hand centre", np.round(SOUP, 4))
else:
    print("eye-in-hand estimate unusable, keeping birdview centre")
if c_agv is not None:
    print("agentview y-centre check: dy =", round(float(c_agv[1] - SOUP[1]), 4))

print("== hover again over refined centre")
assert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=3)

print("== descend to grasp height")
ok = a.move_to([*SOUP, Z_GRASP], q=Q_SOUP, seconds=5, steps=4)
if not ok:
    print("descent blocked; lifting clear")
    p, _ = a.hand_pose()
    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=3, steps=2)
    sys.exit(1)

print("== close gripper")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", gap)
assert gap > 0.02, "closed on air"

print("== lift")
assert a.move_to([*SOUP, Z_CARRY], q=Q_SOUP, seconds=4, steps=3)
print("fingers after lift", a.finger_gap())

print("== move above basket")
assert a.move_to([*BASKET, Z_DROP], seconds=5)
print("fingers above basket", a.finger_gap())

print("== release")
a.gripper(GRIP["open_m"])

print("== retreat up")
a.move_to([*BASKET, Z_DROP + 0.05], seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 43
nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 75; cat /workspace/pick_soup.log

# openrua op 44
sleep 30; tail -6 /workspace/pick_soup.log; pgrep -f pick_soup.py >/dev/null && echo running || echo finished

# openrua op 45
sleep 5; pgrep -f pick_soup.py >/dev/null && echo running || echo finished; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('q now', np.round(a.arm_q(),3))
TILT=np.radians(30); Q_SOUP=quat_mul((0.0,np.sin(TILT/2),0.0,np.cos(TILT/2)),Q_DOWN)
prev=a.arm_q()
for z in (0.58,0.54,0.50,0.48,0.455):
    s=a.solve_ik([-0.2059,-0.1305,z],q=Q_SOUP,seed=prev); print(z,np.round(s,3)); prev=s
"

# openrua op 46
mkdir -p "$(dirname /workspace/probe.py)"
cat > /workspace/probe.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Step the tilted gripper down over the soup can, logging wrench + pose."""
import subprocess

import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *

SOUP = np.array([-0.2100, -0.1310])
TILT = np.radians(30)
Q_SOUP = quat_mul((0.0, np.sin(TILT / 2), 0.0, np.cos(TILT / 2)), Q_DOWN)

a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f = w["m"].wrench.force
    return np.round([f.x, f.y, f.z], 1)


a.gripper(GRIP["open_m"])
for z in (0.62, 0.55, 0.53, 0.51, 0.49, 0.47, 0.455):
    ok = a.move_to([*SOUP, z], q=Q_SOUP, seconds=3, steps=2)
    print(f"z={z} ok={ok} wrench={wrench()} fingers={np.round(a.finger_gap(), 4)}")
    subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", f"eih_{z}.png"],
                   capture_output=True)
    if not ok:
        break
OPENRUA_EOF

# openrua op 47
pgrep -f pick_soup.py >/dev/null && echo "still running" ; nohup python3 -u probe.py > probe.log 2>&1 & sleep 120; cat /workspace/probe.log | grep -E "^z=|off"

# openrua op 48
sleep 45; grep -E "^z=|off|attempt" /workspace/probe.log | tail -8; pgrep -f probe.py >/dev/null && echo running || echo finished

# openrua op 49
pgrep -f probe.py >/dev/null && echo running || echo finished; timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
p,_=a.hand_pose(); print('tcp', np.round(p,4)); print('fingers', a.finger_gap())
P=cloud('robot0_eye_in_hand', a.node)
X,Y,Z=P[...,0],P[...,1],P[...,2]
# can body: cylinder region excluding fingers; fingers are near y = tcp_y +- 0.04
m=(Z>0.44)&(Z<0.53)&(X>p[0]-0.09)&(X<p[0]+0.09)&(Y>p[1]-0.12)&(Y<p[1]+0.12)&np.isfinite(Z)
ys=Y[m]; xs=X[m]; zs=Z[m]
import numpy as np
h,edges=np.histogram(ys,bins=np.arange(p[1]-0.12,p[1]+0.12,0.004))
for c,e in zip(h,edges[:-1]): print(f'{e:+.3f} {c:5d} '+'#'*min(c//20,60))
" 2>&1 | tail -70

# openrua op 50
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
for cam in ('agentview','frontview'):
    P=cloud(cam, a.node)
    X,Y,Z=P[...,0],P[...,1],P[...,2]
    print(cam)
    for z0 in np.arange(0.43,0.53,0.01):
        m=(Z>z0)&(Z<z0+0.01)&(X>-0.26)&(X<-0.15)&(Y>-0.19)&(Y<-0.07)
        if m.sum(): print(f'  z {z0:.2f}-{z0+0.01:.2f} n={m.sum():4d} y=[{Y[m].min():.3f},{Y[m].max():.3f}] width={Y[m].max()-Y[m].min():.3f} x=[{X[m].min():.3f},{X[m].max():.3f}]')
" 2>&1 | grep -v base

# openrua op 51
for c in agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[130:290,140:330],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[150:260,220:340],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_s.png', cv2.resize(im[280:440,300:460],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 52
timeout 900 python3 -u -c "
from arm import *
import rclpy
from geometry_msgs.msg import WrenchStamped
a=Arm()
w={}
a.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: w.__setitem__('m',m),1)
def wrench():
    w.clear()
    while 'm' not in w: rclpy.spin_once(a.node,timeout_sec=0.2)
    f=w['m'].wrench.force; return np.round([f.x,f.y,f.z],1)
TILT=np.radians(30); Q_SOUP=quat_mul((0.0,np.sin(TILT/2),0.0,np.cos(TILT/2)),Q_DOWN)
p,_=a.hand_pose()
print('lift'); a.move_to([p[0],p[1],0.60], q=Q_SOUP, seconds=3, steps=2)
q49=[0.225-0.6, 0.264,-0.526,-2.589,-0.036,2.318,-0.288]   # z=0.49-ish config, joint1 rotated
q455=[0.208-0.6, 0.35,-0.494,-2.575,0.02,2.382,-0.321]
print('free-space config A (joint6=2.32)'); a.move_joints(q49, seconds=5); print(' tcp', np.round(a.hand_pose()[0],3), 'wrench', wrench())
print('free-space config B (joint6=2.38)'); a.move_joints(q455, seconds=3); print(' tcp', np.round(a.hand_pose()[0],3), 'wrench', wrench())
print('joint6 -> 2.6'); a.move_joints([q455[0],q455[1],q455[2],q455[3],q455[4],2.6,q455[6]], seconds=3); print(' wrench', wrench())
" 2>&1 | grep -v base

# openrua op 53
timeout 900 python3 -u -c "
from arm import *
import rclpy
from geometry_msgs.msg import WrenchStamped
a=Arm()
w={}
a.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: w.__setitem__('m',m),1)
def wrench():
    w.clear()
    while 'm' not in w: rclpy.spin_once(a.node,timeout_sec=0.2)
    f=w['m'].wrench.force; return np.round([f.x,f.y,f.z],1)
TILT=np.radians(30); Q_SOUP=quat_mul((0.0,np.sin(TILT/2),0.0,np.cos(TILT/2)),Q_DOWN)
READY=[0,-0.785,0,-2.356,0,1.571,0.785]
print('hover beside can'); a.move_to([-0.21,-0.03,0.60], q=Q_SOUP, seconds=5, seed=READY)
for z in (0.50,0.48,0.455,0.44):
    ok=a.move_to([-0.21,-0.03,z], q=Q_SOUP, seconds=3, steps=2); print(f'z={z} ok={ok} wrench={wrench()}')
    if not ok: break
print('back up'); a.move_to([-0.21,-0.03,0.60], q=Q_SOUP, seconds=3, steps=2)
" 2>&1 | grep -E "^z=|hover|back|attempt 3"

# openrua op 54
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
p,q=a.hand_pose('tcp'); ph,_=a.hand_pose('hand'); print('FK tcp', np.round(p,4), 'hand', np.round(ph,4))
for cam in ('agentview','frontview','sideview'):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(X>-0.30)&(X<-0.08)&(Y>-0.09)&(Y<0.03)&(Z>0.43)&(Z<0.75)&np.isfinite(Z)
    if m.sum()==0: print(cam,'none'); continue
    zs=Z[m]; idx=np.argsort(zs)[:30]
    print(cam, 'lowest gripper points z', np.round(zs[idx].mean(),4), 'at x', np.round(X[m][idx].mean(),3), 'y', np.round(Y[m][idx].mean(),3), ' n', m.sum())
" 2>&1 | grep -v base

# openrua op 55
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
import cv2
a=Arm()
p,q=a.hand_pose('tcp'); ph,_=a.hand_pose('hand')
img=np.zeros((400,400,3),np.uint8)
def px(x,z): return int((x+0.35)*1000), int((0.80-z)*1000)
for cam,col in (('agentview',(255,255,255)),('frontview',(0,255,0)),('sideview',(0,200,255))):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(X>-0.35)&(X<0.05)&(Y>-0.09)&(Y<0.03)&(Z>0.40)&(Z<0.80)&np.isfinite(Z)
    for x,z in zip(X[m][::3],Z[m][::3]):
        u,v=px(x,z); 
        if 0<=u<400 and 0<=v<400: img[v,u]=col
cv2.circle(img,px(p[0],p[2]),4,(0,0,255),-1); cv2.circle(img,px(ph[0],ph[2]),4,(255,0,255),-1)
cv2.line(img,px(-0.35,0.425),px(0.05,0.425),(100,100,100),1)
cv2.imwrite('profile.png',cv2.resize(img,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
print('tcp',np.round(p,3),'hand',np.round(ph,3))
" 2>&1 | grep -v base

# openrua op 56
timeout 600 python3 -u -c "
from arm import *
from seg import cloud
import cv2
a=Arm()
a.gripper(0.04)
print(a.move_to([-0.05,-0.03,0.60], q=Q_DOWN, seconds=5))
p,q=a.hand_pose('tcp'); ph,_=a.hand_pose('hand'); print('q now', np.round(a.arm_q(),3)); print('FK quat', np.round(q,3))
img=np.zeros((400,400,3),np.uint8)
def px(x,z): return int((x+0.25)*1000), int((0.85-z)*1000)
for cam,col in (('agentview',(255,255,255)),('frontview',(0,255,0)),('sideview',(0,200,255))):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(X>-0.25)&(X<0.15)&(Y>-0.09)&(Y<0.03)&(Z>0.45)&(Z<0.85)&np.isfinite(Z)
    for x,z in zip(X[m][::3],Z[m][::3]):
        u,v=px(x,z)
        if 0<=u<400 and 0<=v<400: img[v,u]=col
    zs=Z[m]; idx=np.argsort(zs)[:30]; print(cam,'lowest z',np.round(zs[idx].mean(),4),'x',np.round(X[m][idx].mean(),3),'y',np.round(Y[m][idx].mean(),3))
cv2.circle(img,px(p[0],p[2]),4,(0,0,255),-1); cv2.circle(img,px(ph[0],ph[2]),4,(255,0,255),-1)
cv2.imwrite('profile2.png',cv2.resize(img,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
print('tcp',np.round(p,3),'hand',np.round(ph,3))
" 2>&1 | grep -v base

# openrua op 57
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
p,q=a.hand_pose('tcp'); print('tcp',np.round(p,3),'hand quat',np.round(q,3))
R=quat_to_R(q); print('hand x axis (world)',np.round(R[:,0],3),' hand y axis',np.round(R[:,1],3))
Rl=quat_to_R(Q_DOWN); print('link8 y axis (world)', np.round(Rl[:,1],3))
for cam in ('agentview','frontview','sideview'):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.abs(X-p[0])<0.08)&(np.abs(Y-p[1])<0.08)&(Z>p[2]-0.01)&(Z<p[2]+0.05)&np.isfinite(Z)
    pts=np.stack([X[m],Y[m]],1)-p[:2]
    # cluster by sign along principal axis
    if len(pts)<10: print(cam,'few'); continue
    u,s,vt=np.linalg.svd(pts-pts.mean(0),full_matrices=False); ax=vt[0]
    proj=pts@ax; c1=pts[proj>0].mean(0); c2=pts[proj<=0].mean(0)
    print(cam,'n',len(pts),'finger clusters (rel tcp):',np.round(c1,3),np.round(c2,3),'sep',np.round(np.linalg.norm(c1-c2),3),'dir',np.round(ax,3))
" 2>&1 | grep -v base

# openrua op 58
python3 - <<'EOF'
s=open('/workspace/arm.py').read()
s=s.replace('''def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world Z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)
''','''def q_down_yaw(yaw):
    """Top-down grasp rotated by yaw (rad) about world Z."""
    qz = (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2))
    return quat_mul(qz, Q_DOWN)


# MACHINE FACT (measured from camera clouds): IK/FK poses are for
# panda_link8, and panda_hand is rotated -45 deg about z relative to it,
# so under Q_DOWN the fingers open along world (0.71, -0.71), NOT y.
# q_grasp() hides that: `finger_axis` is the world-plane angle of the
# finger opening axis (0 = fingers along x, pi/2 = along y); `tilt` leans
# the hand about the world axis perpendicular to the fingers (positive =
# fingertips toward -x for finger_axis=pi/2), keeping the pads vertical.
def q_grasp(finger_axis=np.pi / 2, tilt=0.0):
    base = q_down_yaw(finger_axis + np.pi / 4)
    if abs(tilt) < 1e-9:
        return base
    ax = np.array([np.cos(finger_axis), np.sin(finger_axis), 0.0])  # tilt axis = finger axis
    s, c = np.sin(tilt / 2), np.cos(tilt / 2)
    return quat_mul((ax[0] * s, ax[1] * s, ax[2] * s, c), base)


def finger_axis_world(q_link8):
    """World direction along which the fingers open, for a link8 quat."""
    Rz = quat_to_R((0.0, 0.0, np.sin(-np.pi / 8), np.cos(-np.pi / 8)))  # hand = link8 * Rz(-45deg)
    return (quat_to_R(q_link8) @ Rz)[:, 1]
''')
open('/workspace/arm.py','w').write(s)
EOF
timeout 60 python3 -c "
from arm import *
for fa,t in ((np.pi/2,0),(np.pi/2,np.radians(30)),(0,0),(np.pi/2,np.radians(-30))):
    q=q_grasp(fa,t); R=quat_to_R(q)
    print('finger_axis',round(fa,2),'tilt',round(t,2),'-> fingers along',np.round(finger_axis_world(q),3),' link8 z (approach dir)',np.round(R[:,2],3))
"

# openrua op 59
timeout 600 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
print(a.move_to([-0.05,-0.03,0.60], q=q_grasp(np.pi/2,0), seconds=4))
p,q=a.hand_pose('tcp'); print('tcp',np.round(p,3),'predicted finger axis',np.round(finger_axis_world(a.solve_ik([-0.05,-0.03,0.60],q_grasp(np.pi/2,0)) and q_grasp(np.pi/2,0)),3))
for cam in ('agentview','frontview'):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.abs(X-p[0])<0.08)&(np.abs(Y-p[1])<0.08)&(Z>p[2]-0.01)&(Z<p[2]+0.05)&np.isfinite(Z)
    pts=np.stack([X[m],Y[m]],1)-p[:2]
    u,s,vt=np.linalg.svd(pts-pts.mean(0),full_matrices=False); ax=vt[0]
    proj=pts@ax; c1=pts[proj>0].mean(0); c2=pts[proj<=0].mean(0)
    print(cam,'finger clusters (rel tcp):',np.round(c1,3),np.round(c2,3),'sep',np.round(np.linalg.norm(c1-c2),3),'dir',np.round(ax,3))
" 2>&1 | grep -v base

# openrua op 60
timeout 300 python3 -u -c "
from arm import *
a=Arm()
q=np.array(a.arm_q()); print('actual ', np.round(q,3))
s=np.array(a.solve_ik([-0.05,-0.03,0.60],q_grasp(np.pi/2,0))); print('ik sol ', np.round(s,3)); print('diff   ', np.round(s-q,3))
print('limits ', FJT['limits_rad'])
" 2>&1 | grep -v base

# openrua op 61
timeout 600 python3 -u -c "
from arm import *
import rclpy
from geometry_msgs.msg import WrenchStamped
a=Arm()
w={}
a.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: w.__setitem__('m',m),1)
def wrench():
    w.clear()
    while 'm' not in w: rclpy.spin_once(a.node,timeout_sec=0.2)
    f=w['m'].wrench.force; return np.round([f.x,f.y,f.z],1)
q=a.arm_q()
for j7 in (-1.9,-2.2,-2.6, 0.0, 0.8, 1.6, 2.2, 2.6):
    t=list(q); t[6]=j7
    code,err=a.move_joints(t, seconds=3, retries=1)
    print(f'target j7={j7} actual={a.arm_q()[6]:.3f} wrench={wrench()}')
" 2>&1 | grep -E "^target"

# openrua op 62
python3 - <<'EOF'
s=open('/workspace/arm.py').read()
s=s.replace('''    base = q_down_yaw(finger_axis + np.pi / 4)
''','''    yaw = (finger_axis + np.pi / 4 + np.pi / 2) % np.pi - np.pi / 2  # fingers are symmetric: yaw mod pi, nearest 0
    base = q_down_yaw(yaw)
''')
s=s.replace('''        final = np.array(targets[-1], dtype=float)
        for attempt in range(retries):''','''        final = np.array(targets[-1], dtype=float)
        # the controller tracks at roughly 0.2 rad/s; give it the time
        dist = np.abs(final - np.array(self.arm_q())).max()
        seconds = max(seconds, dist / MAX_JOINT_RATE)
        for attempt in range(retries):''')
s=s.replace('''            targets = [final]          # re-send just the final point
            seconds = max(2.0, seconds * 0.6)''','''            targets = [final]          # re-send just the final point
            seconds = max(2.0, err / MAX_JOINT_RATE)''')
s=s.replace('''TCP = float(M["hand"]["tcp_offset_m"])
''','''TCP = float(M["hand"]["tcp_offset_m"])
MAX_JOINT_RATE = 0.15  # rad/s the controller actually achieves (measured)
''')
open('/workspace/arm.py','w').write(s)
EOF
timeout 60 python3 -c "
from arm import *
for fa in (np.pi/2, 0, np.pi/4, -np.pi/2):
    q=q_grasp(fa); print(round(fa,2), 'fingers along', np.round(finger_axis_world(q),2), 'quat', np.round(q,3))
"

# openrua op 63
timeout 900 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
print(a.move_to([-0.05,-0.03,0.60], q=q_grasp(np.pi/2,0), seconds=4))
p,q=a.hand_pose('tcp'); print('tcp',np.round(p,3),'q', np.round(a.arm_q(),3))
for cam in ('agentview','frontview'):
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.abs(X-p[0])<0.08)&(np.abs(Y-p[1])<0.08)&(Z>p[2]-0.01)&(Z<p[2]+0.05)&np.isfinite(Z)
    pts=np.stack([X[m],Y[m]],1)-p[:2]
    u,s,vt=np.linalg.svd(pts-pts.mean(0),full_matrices=False); ax=vt[0]
    proj=pts@ax; c1=pts[proj>0].mean(0); c2=pts[proj<=0].mean(0)
    print(cam,'finger clusters (rel tcp):',np.round(c1,3),np.round(c2,3),'sep',np.round(np.linalg.norm(c1-c2),3),'dir',np.round(ax,3))
" 2>&1 | grep -v base

# openrua op 64
timeout 120 python3 seg.py birdview "soup:-0.32,-0.12,-0.25,-0.05,0.47,0.53" "box:0.05,0.20,-0.28,-0.15,0.44,0.5" "basket:-0.15,0.25,0.15,0.45,0.55,0.7"; timeout 120 python3 seg.py agentview "soup_side:-0.32,-0.12,-0.25,-0.05,0.43,0.46"

# openrua op 65
python3 - <<'EOF'
s=open('pick_soup.py').read()
s=s.replace("SOUP = np.array([-0.2131, -0.1331])          # latest birdview estimate","SOUP = np.array([-0.208, -0.125])            # latest birdview estimate")
s=s.replace('''TILT = np.radians(30)
# rotate Q_DOWN about world Y: fingers point (-sin t, 0, -cos t), i.e.
# down and toward the arm base, so the wrist sits farther out
Q_SOUP = quat_mul((0.0, np.sin(TILT / 2), 0.0, np.cos(TILT / 2)), Q_DOWN)''','''# fingers open along world y; hand leans 30 deg so the fingertips point
# down and toward the arm base (wrist farther out -> joint4 off its limit)
Q_SOUP = q_grasp(np.pi / 2, np.radians(30))''')
s=s.replace("assert a.move_to([*BASKET, Z_DROP], seconds=5)","assert a.move_to([*BASKET, Z_DROP], q=q_grasp(np.pi / 2), seconds=5)")
s=s.replace("a.move_to([*BASKET, Z_DROP + 0.05], seconds=2)","a.move_to([*BASKET, Z_DROP + 0.05], q=q_grasp(np.pi / 2), seconds=2)")
open('pick_soup.py','w').write(s)
EOF
grep -n "q_grasp\|SOUP =" pick_soup.py; nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 100; cat /workspace/pick_soup.log

# openrua op 66
timeout 900 python3 -u -c "
from arm import *
from seg import cloud
import cv2
a=Arm()
SOUP=np.array([-0.2004,-0.1236]); Q=q_grasp(np.pi/2,np.radians(30))
a.move_to([*SOUP,0.50], q=Q, seconds=4, steps=2)
ok=a.move_to([*SOUP,0.455], q=Q, seconds=4, steps=2)
p,q=a.hand_pose('tcp'); ph,_=a.hand_pose('hand'); print('blocked tcp',np.round(p,4),'hand',np.round(ph,4),'ok',ok)
img=np.zeros((300,400,3),np.uint8)
def px(x,z): return int((x+0.35)*1000), int((0.70-z)*1000)
cols={'agentview':(255,255,255),'frontview':(0,255,0),'sideview':(0,200,255)}
slabs={'center':(SOUP[1],0.012),'finger-y':(SOUP[1]-0.04,0.012),'finger+y':(SOUP[1]+0.04,0.012)}
imgs={k:img.copy() for k in slabs}
for cam,col in cols.items():
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    for k,(yc,hw) in slabs.items():
        m=(np.abs(Y-yc)<hw)&(X>-0.35)&(X<0.05)&(Z>0.40)&(Z<0.70)&np.isfinite(Z)
        for x,z in zip(X[m],Z[m]):
            u,v=px(x,z)
            if 0<=u<400 and 0<=v<300: imgs[k][v,u]=col
for k in slabs:
    im=imgs[k]; cv2.circle(im,px(p[0],p[2]),3,(0,0,255),-1); cv2.circle(im,px(ph[0],ph[2]),3,(255,0,255),-1)
    cv2.line(im,px(-0.35,0.425),px(0.05,0.425),(100,100,100),1)
    cv2.imwrite(f'slab_{k}.png',cv2.resize(im,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST))
" 2>&1 | grep -E "blocked|off"

# openrua op 67
mkdir -p "$(dirname /workspace/measure_can.py)"
cat > /workspace/measure_can.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Lift clear of the can, then measure its centre from silhouette edges.

agentview looks along -x -> sees the can's +-y edges; sideview/frontview see
+-x edges. Midpoint of the edges beats a centroid (partial rim, occlusion).
"""
import sys

from arm import *
from seg import cloud, region

Q_SOUP = q_grasp(np.pi / 2, np.radians(30))
Z_HOVER = 0.62

a = Arm()
p, _ = a.hand_pose()
print("tcp now", np.round(p, 4))
if p[2] < Z_HOVER - 0.02:
    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=4, steps=2)

XR, YR, ZR = (-0.27, -0.14), (-0.20, -0.06), (0.435, 0.512)  # can body only, below rim
est = {}
for cam in ("agentview", "sideview", "frontview", "birdview"):
    P = cloud(cam, a.node)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > XR[0]) & (X < XR[1]) & (Y > YR[0]) & (Y < YR[1]) & (Z > ZR[0]) & (Z < ZR[1])
    if m.sum() < 20:
        print(cam, "sees nothing")
        continue
    xs, ys = X[m], Y[m]
    # robust edges: 1st/99th percentile
    xe, ye = np.percentile(xs, [1, 99]), np.percentile(ys, [1, 99])
    est[cam] = dict(n=int(m.sum()), xmid=xe.mean(), ymid=ye.mean(), dx=xe[1] - xe[0], dy=ye[1] - ye[0])
    print(f"{cam}: n={m.sum()} x=[{xe[0]:.4f},{xe[1]:.4f}] (w {xe[1]-xe[0]:.3f})  "
          f"y=[{ye[0]:.4f},{ye[1]:.4f}] (w {ye[1]-ye[0]:.3f})  ztop={Z[m].max():.4f}")
OPENRUA_EOF

# openrua op 68
timeout 300 python3 -u measure_can.py 2>&1 | tail -20

# openrua op 69
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pick_soup.py", "old_string": "SOUP = np.array([-0.208, -0.125])            # latest birdview estimate\\nBASKET = np.array([0.0, 0.255])\\nZ_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.455, 0.72, 0.75", "new_string": "SOUP = np.array([-0.204, -0.1175])           # silhouette-edge midpoint (measure_can.py)\\nBASKET = np.array([0.0, 0.255])\\nZ_HOVER, Z_GRASP, Z_CARRY, Z_DROP = 0.62, 0.46, 0.72, 0.75\\nZ_GOOD_ENOUGH = 0.48   # fingertips sit ~1 cm below FK TCP when tilted; can body is 0.425..0.512\\n\\n\\ndef can_centre(cams=(\\"agentview\\", \\"sideview\\", \\"frontview\\")):\\n    \\"\\"\\"Midpoint of the can\'s x/y silhouette edges (robust to partial rim).\\"\\"\\"\\n    xr, yr, zr = (SOUP[0] - 0.05, SOUP[0] + 0.05), (SOUP[1] - 0.05, SOUP[1] + 0.05), (0.435, 0.512)\\n    xs, ys = [], []\\n    for cam in cams:\\n        P = cloud(cam, a.node)\\n        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]\\n        m = (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zr[0]) & (Z < zr[1])\\n        if m.sum() < 100:\\n            continue\\n        xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])\\n        print(f\\"  {cam}: n={m.sum()} xmid={xe.mean():.4f} (w {xe[1]-xe[0]:.3f}) ymid={ye.mean():.4f} (w {ye[1]-ye[0]:.3f})\\")\\n        if xe[1] - xe[0] > 0.055:\\n            xs.append(xe.mean())\\n        if ye[1] - ye[0] > 0.055:\\n            ys.append(ye.mean())\\n    if not xs or not ys:\\n        return None\\n    return np.array([np.median(xs), np.median(ys)])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 70
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pick_soup.py", "old_string": "print(\\"== re-measure can from eye-in-hand + agentview\\")\\nbox = (SOUP[0] - 0.08, SOUP[0] + 0.08, SOUP[1] - 0.08, SOUP[1] + 0.08)\\nc_eih = region(cloud(\\"robot0_eye_in_hand\\", a.node), \\"eih\\", box[0:2], box[2:4], 0.47, 0.53)\\nc_agv = region(cloud(\\"agentview\\", a.node), \\"agv\\", box[0:2], box[2:4], 0.47, 0.53)\\nif c_eih is not None and np.linalg.norm(c_eih - SOUP) < 0.03:\\n    SOUP = c_eih\\n    print(\\"using eye-in-hand centre\\", np.round(SOUP, 4))\\nelse:\\n    print(\\"eye-in-hand estimate unusable, keeping birdview centre\\")\\nif c_agv is not None:\\n    print(\\"agentview y-centre check: dy =\\", round(float(c_agv[1] - SOUP[1]), 4))\\n\\nprint(\\"== hover again over refined centre\\")\\nassert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=3)\\n\\nprint(\\"== descend to grasp height\\")\\nok = a.move_to([*SOUP, Z_GRASP], q=Q_SOUP, seconds=5, steps=4)\\nif not ok:\\n    print(\\"descent blocked; lifting clear\\")\\n    p, _ = a.hand_pose()\\n    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=3, steps=2)\\n    sys.exit(1)", "new_string": "print(\\"== re-measure can from silhouette edges\\")\\nc = can_centre()\\nif c is not None and np.linalg.norm(c - SOUP) < 0.03:\\n    SOUP = c\\n    print(\\"using measured centre\\", np.round(SOUP, 4))\\nelse:\\n    print(\\"measurement unusable, keeping prior centre\\", c)\\n\\nprint(\\"== hover again over refined centre\\")\\nassert a.move_to([*SOUP, Z_HOVER], q=Q_SOUP, seconds=3)\\n\\nprint(\\"== descend to grasp height\\")\\nok = a.move_to([*SOUP, Z_GRASP], q=Q_SOUP, seconds=5, steps=4)\\np, _ = a.hand_pose()\\nif not ok and p[2] > Z_GOOD_ENOUGH:\\n    print(\\"descent blocked at\\", np.round(p, 4), \\"; lifting clear\\")\\n    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=3, steps=2)\\n    sys.exit(1)\\nprint(\\"grasping at tcp\\", np.round(p, 4))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 71
timeout 900 python3 -u pick_soup.py > pick_soup.log 2>&1; echo exit=$?; tail -40 pick_soup.log

# openrua op 72
mkdir -p "$(dirname /workspace/diag.py)"
cat > /workspace/diag.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""At the blocked pose: which joint can't reach its command? Any contact force?"""
import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *

SOUP = np.array([-0.2043, -0.1178])
Q_SOUP = q_grasp(np.pi / 2, np.radians(30))
LIM = np.array(FJT["limits_rad"])

a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f, t = w["m"].wrench.force, w["m"].wrench.torque
    return np.round([f.x, f.y, f.z], 1), np.round([t.x, t.y, t.z], 1)


a.gripper(GRIP["open_m"])
q = np.array(a.arm_q())
print("joints now     ", np.round(q, 3))
print("dist to limits ", np.round(np.minimum(q - LIM[:, 0], LIM[:, 1] - q), 3))
print("wrench now     ", wrench())
tgt = a.solve_ik([*SOUP, 0.46], q=Q_SOUP, seed=list(q))
print("IK target      ", np.round(tgt, 3) if tgt else None)
if tgt:
    print("target-now     ", np.round(np.array(tgt) - q, 3))
    print("tgt dist limits", np.round(np.minimum(np.array(tgt) - LIM[:, 0], LIM[:, 1] - np.array(tgt)), 3))
p, _ = a.hand_pose()
a.move_to([p[0], p[1], 0.62], q=Q_SOUP, seconds=3, steps=2)
print("wrench at hover", wrench())
OPENRUA_EOF

# openrua op 73
timeout 300 python3 -u diag.py 2>&1 | grep -v "^base"

# openrua op 74
grep -n "def move_joints" -A 40 /workspace/arm.py

# openrua op 75
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        final = np.array(targets[-1], dtype=float)\\n        # the controller tracks at roughly 0.2 rad/s; give it the time\\n        dist = np.abs(final - np.array(self.arm_q())).max()\\n        seconds = max(seconds, dist / MAX_JOINT_RATE)\\n        for attempt in range(retries):", "new_string": "        final = np.array(targets[-1], dtype=float)\\n        cmd = final.copy()   # what we actually send; offset by the sag if needed\\n        # the controller tracks at roughly 0.2 rad/s; give it the time\\n        dist = np.abs(final - np.array(self.arm_q())).max()\\n        seconds = max(seconds, dist / MAX_JOINT_RATE)\\n        for attempt in range(retries):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 76
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "            if err <= tol:\\n                return code, err\\n            targets = [final]          # re-send just the final point\\n            seconds = max(2.0, err / MAX_JOINT_RATE)\\n        return code, err", "new_string": "            if err <= tol:\\n                return code, err\\n            # MACHINE FACT: with the arm low and extended, the controller\\n            # settles short of the command on the gravity-loaded joints\\n            # (2 and 4) with no contact and error_code 0. Re-sending the\\n            # same point does nothing; add the residual to the command.\\n            cmd = cmd + (final - q)\\n            targets = [cmd]\\n            seconds = max(2.0, err / MAX_JOINT_RATE)\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 77
timeout 900 python3 -u pick_soup.py > pick_soup.log 2>&1; echo exit=$?; grep -v "^base" pick_soup.log | tail -40

# openrua op 78
python3 -c "
from arm import *
a=Arm(); a.gripper(GRIP['open_m']); print(np.round(a.arm_q(),3))
" 2>&1 | grep -v ^base
for c in agentview sideview frontview galleryview; do python3 tools/perception/cam_snap.py $c blocked_$c.png >/dev/null 2>&1; done; ls -la blocked_*.png

# openrua op 79
timeout 600 python3 -u -c "
from arm import *
a=Arm()
tgt=np.array([-0.078,0.292,-0.212,-2.584,-0.123,2.362,0.598])
q=np.array(a.arm_q())
# rotate current blocked config about joint1 to free space first (same z), then push to target config there
for name,cfg in [('blocked-rotated', q+np.array([-0.9,0,0,0,0,0,0])), ('target-rotated', tgt+np.array([-0.9,0,0,0,0,0,0]))]:
    code,err=a.move_joints(list(cfg), seconds=6, retries=2)
    print(name, 'err',round(err,4), 'q', np.round(a.arm_q(),3), 'tcp', np.round(a.hand_pose()[0],3))
" 2>&1 | grep -v ^base

# openrua op 80
timeout 120 python3 -u seg.py birdview ketchup:-0.16,0.0,-0.22,-0.03,0.44 soup:-0.26,-0.16,-0.17,-0.06,0.44 tomato:-0.26,-0.14,-0.05,0.10,0.44 2>&1 | grep -v ^base
timeout 120 python3 -u seg.py agentview ketchup:-0.16,0.0,-0.22,-0.03,0.44 2>&1 | grep -v ^base

# openrua op 81
timeout 120 python3 -u -c "
import numpy as np
from seg import cloud
from scipy import ndimage
for cam in ['birdview','agentview']:
    P=cloud(cam); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Z>0.44)&(Z<0.75)&(X>-0.4)&(X<0.4)&(Y>-0.5)&(Y<0.5)
    lab,n=ndimage.label(m)
    print(cam)
    for i in range(1,n+1):
        s=lab==i
        if s.sum()<30: continue
        print(f'  blob n={s.sum()} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}] z=[{Z[s].min():.3f},{Z[s].max():.3f}]')
" 2>&1 | grep -v ^base

# openrua op 82
mkdir -p "$(dirname /workspace/step_descent.py)"
cat > /workspace/step_descent.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Descend over the can in 1 cm steps; log pose error, wrench, can position."""
import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *
from seg import cloud

TILT = np.radians(30)
Q_SOUP = q_grasp(np.pi / 2, TILT)
a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f = w["m"].wrench.force
    return np.round([f.x, f.y, f.z], 1)


def can_from(cam, guess):
    P = cloud(cam, a.node)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (abs(X - guess[0]) < 0.06) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.512)
    if m.sum() < 50:
        return None
    xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])
    return np.array([xe.mean(), ye.mean()]), (xe[1] - xe[0], ye[1] - ye[0])


a.gripper(GRIP["open_m"])
guess = np.array([-0.235, -0.142])
c, wd = can_from("agentview", guess)
c2, wd2 = can_from("sideview", guess)
print("can agentview", np.round(c, 4), np.round(wd, 3), " sideview", np.round(c2, 4), np.round(wd2, 3))
SOUP = (c + c2) / 2
assert a.move_to([*SOUP, 0.62], q=Q_SOUP, seconds=5, seed=[0, -0.785, 0, -2.356, 0, 1.571, 0.785])
print("hover wrench", wrench())
for z in np.arange(0.56, 0.449, -0.01):
    ok = a.move_to([*SOUP, z], q=Q_SOUP, seconds=2, steps=1, tol_m=0.005)
    p, _ = a.hand_pose()
    r = can_from("agentview", SOUP)
    print(f"z={z:.3f} ok={ok} tcp={np.round(p,4)} dxyz={np.round(p-[*SOUP,z],4)} wrench={wrench()} "
          f"can={np.round(r[0],4) if r else None} fingers={np.round(a.finger_gap(),3)}")
    if not ok:
        break
OPENRUA_EOF

# openrua op 83
timeout 900 python3 -u step_descent.py 2>&1 | grep -v "^base\|traj error"

# openrua op 84
grep -n "finger\|hand" /workspace/robot.urdf | grep -n "link name\|joint name\|mesh\|box\|cylinder\|origin\|axis\|limit" | head -60

# openrua op 85
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
p,q=a.hand_pose(); print('tcp',np.round(p,4))
for cam in ['robot0_eye_in_hand','agentview','sideview']:
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    # points near the fingers: within 6cm of tcp in x, z between tcp-0.01 and tcp+0.05, |dy|<0.08, exclude can body (|dy|<0.03)
    m=(abs(X-p[0])<0.05)&(Z>p[2]-0.02)&(Z<p[2]+0.05)&(abs(Y-p[1])<0.08)&np.isfinite(Z)
    dy=Y[m]-p[1]
    h,e=np.histogram(dy,bins=np.arange(-0.08,0.0801,0.005))
    print(cam, 'n',m.sum())
    for c,lo in zip(h,e[:-1]):
        if c: print(f'   dy [{lo:+.3f},{lo+0.005:+.3f}) {c}')
" 2>&1 | grep -v ^base

# openrua op 86
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_blocked.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview side_blocked.png >/dev/null 2>&1; python3 -c "
from PIL import Image
im=Image.open('side_blocked.png'); w,h=im.size; im.crop((int(w*0.35),int(h*0.55),int(w*0.75),int(h*0.95))).resize((800,800)).save('side_crop.png')
im=Image.open('blocked_agentview.png'); w,h=im.size; im.crop((int(w*0.2),int(h*0.25),int(w*0.6),int(h*0.65))).resize((800,800)).save('agv_crop.png')
"

# openrua op 87
timeout 120 python3 -u -c "
from arm import *
from seg import cloud
a=Arm()
p,q=a.hand_pose(); ph,_=a.hand_pose(at='hand'); print('tcp',np.round(p,4),'hand',np.round(ph,4))
for cam in ['agentview','sideview','frontview','robot0_eye_in_hand']:
    P=cloud(cam,a.node); X,Y,Z=P[...,0],P[...,1],P[...,2]
    fin=np.isfinite(Z)
    dy=Y-p[1]
    for side,sgn in (('-y',-1),('+y',1)):
        m=fin&(sgn*dy>0.034)&(sgn*dy<0.06)&(Z>p[2]-0.03)&(Z<p[2]+0.07)&(abs(X-p[0])<0.08)
        if m.sum()>5:
            print(f'{cam} finger{side}: n={m.sum()} x=[{X[m].min():.3f},{X[m].max():.3f}] z=[{Z[m].min():.3f},{Z[m].max():.3f}] |dy|=[{abs(dy[m]).min():.3f},{abs(dy[m]).max():.3f}]')
    m=fin&(abs(dy)<0.033)&(Z>0.44)&(Z<0.52)&(abs(X-p[0])<0.08)
    if m.sum()>5:
        print(f'{cam} can: n={m.sum()} x=[{X[m].min():.3f},{X[m].max():.3f}] z=[{Z[m].min():.3f},{Z[m].max():.3f}] dy=[{dy[m].min():.3f},{dy[m].max():.3f}]')
" 2>&1 | grep -v ^base

# openrua op 88
grep -n "def q_grasp" -B2 -A12 /workspace/arm.py | head -30; grep -n "def hand_pose" -A 14 /workspace/arm.py

# openrua op 89
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "def finger_axis_world(q_link8):", "new_string": "def R_to_quat(R):\\n    \\"\\"\\"3x3 rotation -> quaternion (x,y,z,w).\\"\\"\\"\\n    t = np.trace(R)\\n    if t > 0:\\n        s = np.sqrt(t + 1.0) * 2\\n        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)\\n    i = int(np.argmax(np.diag(R)))\\n    j, k = (i + 1) % 3, (i + 2) % 3\\n    s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2\\n    q = [0.0, 0.0, 0.0, 0.0]\\n    q[i] = 0.25 * s\\n    q[j] = (R[j, i] + R[i, j]) / s\\n    q[k] = (R[k, i] + R[i, k]) / s\\n    q[3] = (R[k, j] - R[j, k]) / s\\n    return tuple(q)\\n\\n\\ndef q_from_axes(finger_axis, approach_axis):\\n    \\"\\"\\"link8 quaternion for a hand whose fingers open along `finger_axis`\\n    (world) and whose fingertips point along `approach_axis` (world).\\n    e.g. q_from_axes((1,0,0), (0,1,0)) = horizontal hand pointing +y,\\n    fingers closing along x. Accounts for the hand\'s -45deg offset.\\"\\"\\"\\n    z = np.array(approach_axis, float); z /= np.linalg.norm(z)\\n    y = np.array(finger_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)\\n    x = np.cross(y, z)\\n    R_hand = np.column_stack([x, y, z])\\n    Rz45 = quat_to_R((0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))  # hand = link8*Rz(-45) -> link8 = hand*Rz(+45)\\n    return R_to_quat(R_hand @ Rz45)\\n\\n\\ndef finger_axis_world(q_link8):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 90
timeout 120 python3 -u -c "
from arm import *
a=Arm()
QH=q_from_axes((1,0,0),(0,1,0))
print('QH',np.round(QH,4),'finger axis',np.round(finger_axis_world(QH),3),'approach',np.round(quat_to_R(QH)[:,2],3))
# sanity: q_grasp(pi/2) should equal q_from_axes((0,1,0),(0,0,-1)) up to sign
print('check', np.round(q_grasp(np.pi/2),4), np.round(q_from_axes((0,1,0),(0,0,-1)),4))
cx,cy=-0.234,-0.14
q=a.arm_q()
for name,pos in [('high',(cx,cy-0.10,0.62)),('pre',(cx,cy-0.10,0.49)),('grasp',(cx,cy,0.49)),('lift',(cx,cy,0.72)),('basket',(0.0,0.255,0.75))]:
    s=a.solve_ik(pos,q=QH,seed=q)
    print(name,pos, None if s is None else np.round(s,3))
    if s: q=s
" 2>&1 | grep -v ^base

# openrua op 91
timeout 200 python3 -u -c "
from arm import *
a=Arm()
cx,cy=-0.234,-0.14
seeds=[[-0.7,0.3,0,-2.0,0,2.3,0.0],[-0.7,0.3,0,-2.0,0,2.3,-1.5],[-0.7,0.3,0,-2.0,0,2.3,1.5],[-1.0,0.5,0.5,-1.8,-1.5,2.0,0.8],[-0.7,0.0,0.0,-2.4,1.5,2.5,0.0],[-0.5,0.4,-0.8,-2.0,1.0,2.6,-0.8],[-0.5,0.4,0.8,-2.0,-1.0,2.6,0.8]]
for ax in [(0,1,0),(-0.3,1,0),(-0.6,1,0),(-1,1,0)]:
    QH=q_from_axes((1,0,0),ax)
    d=np.array(ax,float); d/=np.linalg.norm(d)
    pre=np.array([cx,cy,0.49])-0.10*d
    best=None
    for s in seeds:
        sol=a.solve_ik(pre,q=QH,seed=s)
        if sol is None: continue
        sol=np.array(sol)
        score=abs(sol[0]+0.6)+abs(sol[2])+abs(sol[4])  # prefer untwisted arm
        if best is None or score<best[0]: best=(score,sol)
    print('approach',ax,'pre',np.round(pre,3),'->', None if best is None else np.round(best[1],3))
" 2>&1 | grep -v ^base

# openrua op 92
mkdir -p "$(dirname /workspace/pick_soup_h.py)"
cat > /workspace/pick_soup_h.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the alphabet soup can with a HORIZONTAL grasp and drop it in the basket.

MACHINE FACT: this gripper's pads are ~26 mm thick, inner faces only +-34 mm
from the TCP when open (68 mm gap). The can body is 62 mm, its rim 66 mm, so a
top-down straddle catches the rim. Instead approach from -y with the hand
horizontal (axis +y), fingers closing along x on the body below the rim.
"""
import sys

from arm import *
from seg import cloud

BASKET = np.array([0.0, 0.255])
Z_G = 0.485          # pads ~0.475..0.495: below rim (0.512), hand clear of table (0.425)
APPROACH = 0.10      # start this far short of the can along -y
QH = q_from_axes((1, 0, 0), (0, 1, 0))          # fingers along x, fingertips toward +y
QV = q_grasp(np.pi / 2)                          # vertical hand, fingers along y
SEED_H = [-0.7, 0.3, 0, -2.0, 0, 2.3, 0.0]
Z_HOVER = 0.62

a = Arm()


def can_centre(guess, cams=("agentview", "sideview", "frontview")):
    xs, ys = [], []
    for cam in cams:
        P = cloud(cam, a.node)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
        m = (abs(X - guess[0]) < 0.06) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.512)
        if m.sum() < 100:
            continue
        xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])
        print(f"  {cam}: n={m.sum()} xmid={xe.mean():.4f} (w {xe[1]-xe[0]:.3f}) ymid={ye.mean():.4f} (w {ye[1]-ye[0]:.3f})")
        if xe[1] - xe[0] > 0.055:
            xs.append(xe.mean())
        if ye[1] - ye[0] > 0.055:
            ys.append(ye.mean())
    return np.array([np.median(xs), np.median(ys)]) if xs and ys else None


def extent(name, box):
    """z/x/y extents of cloud points inside a world box (from agentview+sideview)."""
    pts = []
    for cam in ("agentview", "sideview"):
        P = cloud(cam, a.node).reshape(-1, 3)
        m = np.all(np.isfinite(P), 1)
        for i in range(3):
            m &= (P[:, i] > box[i][0]) & (P[:, i] < box[i][1])
        pts.append(P[m])
    pts = np.concatenate(pts)
    if len(pts):
        print(f"  {name}: n={len(pts)} x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z=[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    return pts


print("== open, clear the can")
a.gripper(GRIP["open_m"])
p, _ = a.hand_pose()
if p[2] < Z_HOVER - 0.02:
    a.move_to([p[0], p[1], Z_HOVER], q=q_grasp(np.pi / 2, np.radians(30)), seconds=3, steps=2)

print("== measure can")
SOUP = can_centre(np.array([-0.234, -0.14]))
assert SOUP is not None, "can not found"
print("can centre", np.round(SOUP, 4))

print("== high pre-pose, hand horizontal pointing +y")
pre_xy = SOUP - [0, APPROACH]
assert a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=6, seed=SEED_H)
print("  finger axis now", np.round(finger_axis_world(a.hand_pose()[1]), 3))
p, _ = a.hand_pose()
print("  gripper extents at high pose (tcp", np.round(p, 3), ")")
extent("fingers", [(p[0] - 0.08, p[0] + 0.08), (p[1] - 0.03, p[1] + 0.03), (p[2] - 0.08, p[2] + 0.08)])
extent("hand", [(p[0] - 0.15, p[0] + 0.15), (p[1] - 0.20, p[1] - 0.06), (p[2] - 0.12, p[2] + 0.12)])

print("== descend beside the can")
assert a.move_to([*pre_xy, Z_G], q=QH, seconds=5, steps=3, tol_m=0.006)

print("== slide +y onto the can body")
ok = a.move_to([*SOUP, Z_G], q=QH, seconds=5, steps=4, tol_m=0.006)
p, _ = a.hand_pose()
if not ok:
    print("slide blocked at", np.round(p, 4), "-> backing off")
    a.move_to([*pre_xy, Z_G], q=QH, seconds=3, steps=2)
    a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=3, steps=2)
    sys.exit(1)

print("== close")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", round(gap, 4))
if gap < 0.045:
    print("did not grip the can; opening and backing off")
    a.gripper(GRIP["open_m"])
    a.move_to([*pre_xy, Z_G], q=QH, seconds=3, steps=2)
    a.move_to([*pre_xy, Z_HOVER], q=QH, seconds=3, steps=2)
    sys.exit(1)

print("== lift")
assert a.move_to([*SOUP, 0.72], q=QH, seconds=4, steps=3)
print("fingers after lift", np.round(a.finger_gap(), 4))

print("== to basket")
cur = a.arm_q()
done = False
for q in (QH, q_from_axes((1, 0, 0), (0.7, 1, 0)), q_from_axes((0, 1, 0), (1, 0, 0)), QV):
    sol = a.solve_ik([*BASKET, 0.75], q=q, seed=cur)
    if sol is not None and abs(sol[0] - 0.46) < 0.8 and abs(sol[2]) < 1.6:
        print("  basket orientation", np.round(q, 3), "joints", np.round(sol, 3))
        a.move_joints(sol, seconds=6)
        done = True
        break
assert done, "no basket IK"
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.03 and abs(p[1] - BASKET[1]) < 0.03

print("== release")
a.gripper(GRIP["open_m"])
a.move_to([p[0], p[1], p[2] + 0.05], q=a.hand_pose()[1], seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 93
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pick_soup_h.py", "old_string": "a.move_to([p[0], p[1], p[2] + 0.05], q=a.hand_pose()[1], seconds=2)", "new_string": "a.move_to([p[0], p[1], p[2] + 0.05], q=q, seconds=2)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 94
timeout 1200 python3 -u pick_soup_h.py > pick_h.log 2>&1; echo exit=$?; grep -v "^base" pick_h.log

# openrua op 95
timeout 300 python3 -u -c "
from arm import *
a=Arm()
cur=a.arm_q(); print('cur',np.round(cur,3),'fingers',np.round(a.finger_gap(),3))
B=(0.0,0.255,0.78)
seeds=[cur,[0.46,0.5,0,-1.8,0,2.3,-2.3],[0.46,0.5,0,-1.8,0,2.3,-0.8],[0.46,0.5,0,-1.8,0,2.3,0.8],[0.46,0.2,0,-2.2,0,2.4,0.0],[0.8,0.6,-0.5,-1.6,0.5,2.2,-1.5]]
for deg in [0,30,60,90,120,150]:
    ph=np.radians(deg); ap=(np.cos(ph),np.sin(ph),0); fa=(-np.sin(ph),np.cos(ph),0)
    q=q_from_axes(fa,ap)
    for s in seeds:
        sol=a.solve_ik(B,q=q,seed=s)
        if sol is not None:
            print(f'approach {deg:3d}deg sol', np.round(sol,3))
            break
    else:
        print(f'approach {deg:3d}deg none')
" 2>&1 | grep -v ^base

# openrua op 96
mkdir -p "$(dirname /workspace/carry_soup.py)"
cat > /workspace/carry_soup.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the (already grasped) can to the basket and release it."""
from arm import *

BASKET = np.array([0.0, 0.255])
Z_CARRY = 0.80
QH = q_from_axes((1, 0, 0), (0, 1, 0))
ph = np.radians(150)
QB = q_from_axes((-np.sin(ph), np.cos(ph), 0), (np.cos(ph), np.sin(ph), 0))  # hand horizontal, pointing 150deg


def slerp(q0, q1, t):
    q0, q1 = np.array(q0, float), np.array(q1, float)
    d = q0.dot(q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        r = q0 + t * (q1 - q0)
        return tuple(r / np.linalg.norm(r))
    th = np.arccos(d)
    return tuple((np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th))


a = Arm()
g = a.finger_gap()
print("fingers", np.round(g, 4))
assert g[0] + abs(g[1]) > 0.045, "not holding the can"
p, _ = a.hand_pose()
print("tcp", np.round(p, 4))

print("== up to carry height")
assert a.move_to([p[0], p[1], Z_CARRY], q=QH, seconds=3, steps=2)
p, _ = a.hand_pose()

print("== rotate hand in place (slerp waypoints)")
seed = a.arm_q()
wps = []
for t in np.linspace(0, 1, 7)[1:]:
    q = slerp(QH, QB, t)
    sol = a.solve_ik(p, q=q, seed=seed)
    assert sol is not None, f"no IK at t={t}"
    print(f"  t={t:.2f} joints {np.round(sol,3)}")
    wps.append(sol)
    seed = sol
a.move_joints(wps, seconds=8)
p2, _ = a.hand_pose()
print("  tcp after rotate", np.round(p2, 4), "fingers", np.round(a.finger_gap(), 4))

print("== translate to basket")
assert a.move_to([*BASKET, Z_CARRY], q=QB, seconds=6, steps=4, tol_m=0.02)
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.04 and abs(p[1] - BASKET[1]) < 0.04

print("== release")
a.gripper(GRIP["open_m"])
print("fingers", np.round(a.finger_gap(), 4))
a.move_to([p[0], p[1], p[2] + 0.05], q=QB, seconds=2)
print("DONE soup")
OPENRUA_EOF

# openrua op 97
timeout 900 python3 -u carry_soup.py 2>&1 | grep -v "^base"

# openrua op 98
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('joints',np.round(a.arm_q(),3)); print('fingers',np.round(a.finger_gap(),4)); print('tcp',np.round(a.hand_pose()[0],4))
" 2>&1 | grep -v ^base
for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c now_$c.png >/dev/null 2>&1; done

# openrua op 99
python3 tools/perception/cam_snap.py sideview now_sideview.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py robot0_eye_in_hand now_eih.png >/dev/null 2>&1; echo ok

# openrua op 100
timeout 600 python3 -u -c "
from arm import *
a=Arm()
p,qh=a.hand_pose()
# link8 = hand * Rz(+45deg)
q8=quat_mul(qh,(0,0,np.sin(np.pi/8),np.cos(np.pi/8)))
# sanity: IK at current pose with this orientation should return ~current joints
cur=a.arm_q(); sol=a.solve_ik(p,q=q8,seed=cur)
print('cur',np.round(cur,3)); print('ik ',None if sol is None else np.round(sol,3))
assert sol is not None and np.abs(np.array(sol)-cur).max()<0.05, 'orientation reconstruction mismatch'
print('finger axis', np.round(finger_axis_world(q8),3), 'approach', np.round(quat_to_R(q8)[:,2],3))
ok=a.move_to([0.0,0.255,0.78],q=q8,seconds=5,steps=3,tol_m=0.015)
p,_=a.hand_pose(); print('ok',ok,'tcp',np.round(p,4),'fingers',np.round(a.finger_gap(),4))
if ok or (abs(p[0])<0.04 and abs(p[1]-0.255)<0.04 and p[2]>0.70):
    a.gripper(GRIP['open_m'])
    print('released; fingers',np.round(a.finger_gap(),4))
    a.move_to([p[0],p[1],p[2]+0.06],q=q8,seconds=2,steps=2)
else:
    print('NOT releasing')
" 2>&1 | grep -v ^base

# openrua op 101
timeout 120 python3 -u -c "
import numpy as np
from seg import cloud
from scipy import ndimage
for cam in ['agentview','frontview']:
    P=cloud(cam); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Z>0.44)&(Z<0.62)&(X>-0.12)&(X<0.12)&(Y>0.12)&(Y<0.40)
    lab,n=ndimage.label(m)
    print(cam)
    for i in range(1,n+1):
        s=lab==i
        if s.sum()<30: continue
        print(f'  blob n={s.sum()} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}] z=[{Z[s].min():.3f},{Z[s].max():.3f}]')
" 2>&1 | grep -v ^base
for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c after_$c.png >/dev/null 2>&1; done

# openrua op 102
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "def finger_axis_world(q_link8):", "new_string": "def slerp(q0, q1, t):\\n    q0, q1 = np.array(q0, float), np.array(q1, float)\\n    d = q0.dot(q1)\\n    if d < 0:\\n        q1, d = -q1, -d\\n    if d > 0.9995:\\n        r = q0 + t * (q1 - q0)\\n        return tuple(r / np.linalg.norm(r))\\n    th = np.arccos(d)\\n    return tuple((np.sin((1 - t) * th) * q0 + np.sin(t * th) * q1) / np.sin(th))\\n\\n\\ndef finger_axis_world(q_link8):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 103
grep -n "    def servo" -B3 /workspace/arm.py | head; grep -n "def move_to" -A 40 /workspace/arm.py | grep -n "return" 

# openrua op 104
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def servo(self, v, n=20):", "new_string": "    def link8_quat(self):\\n        \\"\\"\\"Current link8 orientation (hand = link8 * Rz(-45deg)).\\"\\"\\"\\n        _, qh = self.hand_pose()\\n        return quat_mul(qh, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))\\n\\n    def move_pose(self, pos_world, q, steps=6, seconds=6.0, max_jump=0.6, tol_m=0.01):\\n        \\"\\"\\"Move TCP to (pos, link8 quat) interpolating position linearly and\\n        orientation by slerp from the current pose; IK per waypoint seeded\\n        by the previous one. MACHINE FACT: KDL IK flips branches without\\n        warning, which produced a wild swing once - so refuse if two\\n        consecutive waypoints differ by more than max_jump rad.\\"\\"\\"\\n        p0, _ = self.hand_pose()\\n        q0 = self.link8_quat()\\n        target = np.array(pos_world, float)\\n        seed = self.arm_q()\\n        wps = []\\n        for t in np.linspace(0, 1, steps + 1)[1:]:\\n            sol = self.solve_ik(p0 + t * (target - p0), q=slerp(q0, q, t), seed=seed)\\n            if sol is None:\\n                print(f\\"  move_pose: no IK at t={t:.2f}\\")\\n                return False\\n            jump = np.abs(np.array(sol) - np.array(seed)).max()\\n            if jump > max_jump:\\n                print(f\\"  move_pose: branch jump {jump:.2f} rad at t={t:.2f}; refusing\\")\\n                return False\\n            wps.append(sol)\\n            seed = sol\\n        self.move_joints(wps, seconds=seconds)\\n        p, _ = self.hand_pose()\\n        d = np.linalg.norm(p - target)\\n        print(f\\"  now at tcp {np.round(p, 4)} (target {np.round(target, 4)}, off {d*1000:.1f} mm)\\")\\n        return d <= tol_m\\n\\n    def servo(self, v, n=20):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 105
mkdir -p "$(dirname /workspace/pick_box.py)"
cat > /workspace/pick_box.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the cream cheese box (8x4x3 cm, long axis along x) and drop it in the basket."""
import sys

from arm import *
from seg import cloud

BASKET = np.array([0.0, 0.255])
QV = q_grasp(np.pi / 2)        # vertical hand, fingers close along y (the 4 cm side)
Z_HOVER, Z_GRASP, Z_CARRY = 0.60, 0.437, 0.80
a = Arm()


def box_centre(guess):
    """Centroid of the box top from birdview + agentview (it is only 3 cm tall)."""
    cs = []
    for cam in ("birdview", "agentview"):
        P = cloud(cam, a.node)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
        m = (abs(X - guess[0]) < 0.07) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.47)
        if m.sum() < 30:
            continue
        xe, ye = np.percentile(X[m], [2, 98]), np.percentile(Y[m], [2, 98])
        print(f"  {cam}: n={m.sum()} x=[{xe[0]:.3f},{xe[1]:.3f}] y=[{ye[0]:.3f},{ye[1]:.3f}] ztop={Z[m].max():.4f}")
        cs.append([xe.mean(), ye.mean()])
    return np.mean(cs, 0) if cs else None


print("== measure box")
BOX = box_centre(np.array([0.116, -0.209]))
assert BOX is not None
print("box centre", np.round(BOX, 4))

print("== open; go high then over to the box hover (continuity-checked)")
a.gripper(GRIP["open_m"])
p, _ = a.hand_pose()
assert a.move_pose([p[0], p[1], 0.90], a.link8_quat(), steps=3, seconds=3)
if not a.move_pose([*BOX, Z_HOVER + 0.15], QV, steps=8, seconds=10, tol_m=0.02):
    # fall back: plain IK move but only if it is a moderate joint move
    sol = a.solve_ik([*BOX, Z_HOVER + 0.15], q=QV, seed=[0, -0.5, 0, -2.0, 0, 1.8, 0.785])
    assert sol is not None
    print("  fallback joint move, joints", np.round(sol, 3))
    a.move_joints(sol, seconds=8)
assert a.move_to([*BOX, Z_HOVER], q=QV, seconds=3, steps=2)
print("  finger axis", np.round(finger_axis_world(a.link8_quat()), 3))

print("== re-measure and centre")
c = box_centre(BOX)
if c is not None and np.linalg.norm(c - BOX) < 0.03:
    BOX = c
print("box centre", np.round(BOX, 4))
assert a.move_to([*BOX, Z_HOVER], q=QV, seconds=2)

print("== descend")
ok = a.move_to([*BOX, Z_GRASP], q=QV, seconds=5, steps=4, tol_m=0.006)
p, _ = a.hand_pose()
if not ok and p[2] > 0.45:
    print("blocked at", np.round(p, 4), "lifting")
    a.move_to([p[0], p[1], Z_HOVER], q=QV, seconds=3, steps=2)
    sys.exit(1)

print("== close")
g = a.gripper(GRIP["closed_m"])
gap = g[0] + abs(g[1])
print("finger gap (sum) =", round(gap, 4))
if not 0.03 < gap < 0.055:
    print("no box in hand; opening and lifting")
    a.gripper(GRIP["open_m"])
    a.move_to([*BOX, Z_HOVER], q=QV, seconds=3, steps=2)
    sys.exit(1)

print("== lift")
assert a.move_to([*BOX, Z_CARRY], q=QV, seconds=4, steps=3)
print("fingers after lift", np.round(a.finger_gap(), 4))

print("== to basket")
if not a.move_pose([*BASKET, Z_CARRY], QV, steps=8, seconds=10, tol_m=0.02):
    sys.exit("basket move refused")
assert a.move_to([*BASKET, 0.72], q=QV, seconds=3, steps=2, tol_m=0.02)
p, _ = a.hand_pose()
print("  tcp above basket", np.round(p, 4), "fingers", np.round(a.finger_gap(), 4))
assert abs(p[0] - BASKET[0]) < 0.04 and abs(p[1] - BASKET[1]) < 0.04

print("== release")
a.gripper(GRIP["open_m"])
a.move_to([*BASKET, Z_CARRY], q=QV, seconds=2, steps=2)
print("DONE box")
OPENRUA_EOF

# openrua op 106
timeout 1500 python3 -u pick_box.py > pick_box.log 2>&1; echo exit=$?; grep -v "^base" pick_box.log

# openrua op 107
timeout 120 python3 -u -c "
import numpy as np
from seg import cloud
from scipy import ndimage
for cam in ['agentview','birdview']:
    P=cloud(cam); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Z>0.435)&(Z<0.62)&(X>-0.45)&(X<0.45)&(Y>-0.5)&(Y<0.5)
    lab,n=ndimage.label(m)
    print(cam)
    for i in range(1,n+1):
        s=lab==i
        if s.sum()<30: continue
        print(f'  blob n={s.sum()} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}] z=[{Z[s].min():.3f},{Z[s].max():.3f}]')
" 2>&1 | grep -v ^base
for c in agentview frontview sideview; do python3 tools/perception/cam_snap.py $c final_$c.png >/dev/null 2>&1; done

# openrua op 108
python3 tools/perception/cam_snap.py robot0_eye_in_hand final_eih.png >/dev/null 2>&1; timeout 120 python3 -u -c "
import numpy as np
from seg import cloud
P=cloud('robot0_eye_in_hand'); X,Y,Z=P[...,0],P[...,1],P[...,2]
inside=(X>-0.075)&(X<0.08)&(Y>0.17)&(Y<0.335)&np.isfinite(Z)
print('basket floor/contents z percentiles', np.round(np.percentile(Z[inside],[1,50,90,99]),3))
for name,zr in [('floor',(0.43,0.45)),('can/box heights',(0.45,0.56))]:
    m=inside&(Z>zr[0])&(Z<zr[1])
    if m.sum(): print(f'  {name}: n={m.sum()} x=[{X[m].min():.3f},{X[m].max():.3f}] y=[{Y[m].min():.3f},{Y[m].max():.3f}] z=[{Z[m].min():.3f},{Z[m].max():.3f}]')
" 2>&1 | grep -v ^base

# openrua op 109
ls /claude-config/projects/-workspace/memory/ 2>/dev/null; cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null

# openrua op 110
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-facts.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-facts.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-facts
description: Non-obvious facts about the simulated Panda in /workspace that contradict or extend its docs (IK frame, gripper gap, controller behaviour)
metadata:
  type: project
---

Facts learned the hard way on the /workspace Panda sim (Sept 2026), all verified by sensors:

- MoveIt `/compute_ik` and `/compute_fk` poses (empty frame_id) are already in the WORLD frame; do not add the world->panda_link0 offset (-0.51, 0, 0.42).
- IK tip is `panda_link8`; `panda_hand` is rotated -45 deg about z, so the fingers do not open along link8's y. Use `q_grasp` / `q_from_axes` in /workspace/arm.py.
- Gripper pads are ~26 mm thick: inner faces are only +-34 mm from the TCP when open (68 mm gap), not 80 mm. Objects wider than ~60 mm (the 62/66 mm soup can) cannot be straddled top-down; grasp them horizontally from the side below the rim.
- The external wrench topic does NOT register fingertip-vs-object shoving; the can was pushed 1.4 cm with baseline wrench. Verify contact by re-measuring object positions from cameras.
- The trajectory controller lags (~0.15 rad/s) and returns -5 while still moving; settle and re-send. If the residual does not shrink on re-send, the arm is physically blocked - test the same joint config rotated about joint1 in free space to tell self/environment collision apart.
- KDL IK flips solution branches silently between nearby waypoints; always check consecutive joint jumps (arm.py `move_pose` guard) before sending a multi-waypoint trajectory.

**Why:** the generic docs describe an 80 mm gripper, base-frame IK, and a responsive controller; none of that held here and each cost many attempts.

**How to apply:** reuse /workspace/arm.py, seg.py, pick_soup_h.py, pick_box.py; measure object centres from silhouette edges across agentview/sideview/frontview rather than birdview centroids.
OPENRUA_EOF

# openrua op 111
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim machine facts](panda-sim-machine-facts.md) — world-frame IK, 68 mm real gripper gap, wrench blind to shoving, KDL branch flips
OPENRUA_EOF
