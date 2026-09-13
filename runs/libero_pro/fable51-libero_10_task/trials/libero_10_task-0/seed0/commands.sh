#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v -E "parameter|describe|get_type" | head -40; echo ----; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_cc.png', cv2.resize(im[180:280,60:170],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_can.png', cv2.resize(im[20:140,310:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_bluecan.png', cv2.resize(im[40:120,140:220],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_bottle.png', cv2.resize(im[0:90,170:250],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_bluecan2.png', cv2.resize(im[220:310,180:250],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_can2.png', cv2.resize(im[220:310,315:385],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 7
timeout 60 python3 tools/perception/px2world.py robot0_robotview 110 230; timeout 60 python3 tools/perception/px2world.py robot0_robotview 355 60; timeout 60 python3 tools/perception/px2world.py robot0_robotview 355 100; timeout 60 python3 tools/perception/px2world.py robot0_robotview 560 100

# openrua op 8
cat > pc.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('pc')
buf=Buffer(); TransformListener(buf,node)
depth=grab(node,f'/{cam}/depth/image_raw',Image)
color=grab(node,f'/{cam}/color/image_raw',Image)
info=grab(node,f'/{cam}/color/camera_info',CameraInfo)
D=CvBridge().imgmsg_to_cv2(depth,'passthrough').astype(np.float64)
C=CvBridge().imgmsg_to_cv2(color,'bgr8')
frame=f'{cam}_optical_frame'
while not buf.can_transform('world',frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform('world',frame,rclpy.time.Time())
q=t.transform.rotation; R=qR(q.x,q.y,q.z,q.w)
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(u-cx)*D/fx,(v-cy)*D/fy,D],-1)
Wp=P@R.T+tr
np.save(f'{cam}_world.npy',Wp)
print('cam pos',tr, 'depth range',np.nanmin(D),np.nanmax(D))
print('K',fx,fy,cx,cy,'size',W,H)
rclpy.shutdown()
EOF
timeout 120 python3 pc.py robot0_robotview; timeout 120 python3 pc.py birdview

# openrua op 9
cat > seg.py <<'EOF'
import numpy as np, cv2
Wp=np.load('robot0_robotview_world.npy')
C=cv2.imread('robot0_robotview.png')
z=Wp[...,2]
# table height: mode of z
hist,edges=np.histogram(z[np.isfinite(z)],bins=200)
tz=edges[np.argmax(hist)]; print('table z ~',tz)
mask=(z>tz+0.01)&np.isfinite(z)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<150: continue
    m=lab==i
    pts=Wp[m]
    print(i,'px area',stats[i,4],'centroid px',cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3),'mean',pts.mean(0).round(3))
EOF
python3 seg.py

# openrua op 10
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; ros2 topic echo /tf_static --once 2>/dev/null | grep -A3 -E "frame_id" | head -60

# openrua op 11
cat > seg2.py <<'EOF'
import numpy as np, cv2
Wp=np.load('birdview_world.npy')
z=Wp[...,2]
mask=((z>0.44)&(z<0.75)&np.isfinite(z)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<20: continue
    pts=Wp[lab==i]
    print(i,'px area',stats[i,4],'centroid px',cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
EOF
python3 seg2.py

# openrua op 12
python3 -c "
import numpy as np
Wp=np.load('birdview_world.npy')
m=(np.abs(Wp[...,0]+0.10)<0.06)&(np.abs(Wp[...,1]-0.043)<0.06)
pts=Wp[m]; print(pts.shape, pts[:,2].min(), pts[:,2].max())
import cv2
vs,us=np.where(m); print(us.min(),us.max(),vs.min(),vs.max())
hi=pts[pts[:,2]>0.5]; print('top pts', hi.shape, hi.mean(0) if len(hi) else None, hi[:,0].min(), hi[:,0].max(), hi[:,1].min(), hi[:,1].max())
"


# openrua op 13
python3 -c "
import numpy as np, cv2
Wp=np.load('robot0_robotview_world.npy')
z=Wp[...,2]
m=(z>0.437)&(z<0.50)&(Wp[...,0]>0.05)&(Wp[...,0]<0.2)&(Wp[...,1]<-0.12)&(Wp[...,1]>-0.30)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<100: continue
    pts=Wp[lab==i]
    print(i,stats[i,4],cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print('  top face mean',top.mean(0).round(3),'n',len(top))
    # PCA of top face for orientation
    xy=top[:,:2]-top[:,:2].mean(0)
    w,v=np.linalg.eigh(xy.T@xy)
    print('  axes',v.round(3),'sqrt eig',np.sqrt(w/len(top)).round(3))
"


# openrua op 14
ros2 topic echo /pipeline_state --once 2>&1 | head -20; ros2 topic info /pipeline_state

# openrua op 15
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot control helpers: FK, IK, trajectory, gripper, joint state.

Coordinates in this module are WORLD frame unless suffixed _b (base frame).
world -> panda_link0 is a pure translation WB = (-0.51, 0, 0.42).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

WB = np.array([-0.51, 0.0, 0.42])
M = yaml.safe_load(open('/workspace/machine.yaml'))
TRAJ = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = TRAJ['joints']
LIMITS = TRAJ['limits_rad']
TCP = M['hand']['tcp_offset_m']
# top-down grasp, fingers close along world Y (hand x = world x)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_yaw_down(yaw):
    """Hand pointing down (z down), rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy)
    # product (w1,v1)*(w2,v2): w = w1w2 - v1.v2 ; v = w1 v2 + w2 v1 + v1 x v2
    w1, v1 = cy, np.array([0, 0, sy])
    w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('ctl')
        self._js = {}
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self._js.__setitem__('m', m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik = self.node.create_client(GetPositionIK, M['planning']['ik_service'])
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        assert self.fjt.wait_for_server(timeout_sec=20)
        assert self.grip.wait_for_server(timeout_sec=20)
        assert self.ik.wait_for_service(timeout_sec=20)
        assert self.fk.wait_for_service(timeout_sec=20)

    def joints(self):
        self._js.pop('m', None)
        while 'm' not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j['panda_finger_joint1'] - j['panda_finger_joint2']

    def _arm_state(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(x) for x in q]
        return js

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (xyz, quat xyzw)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        req.robot_state.joint_state = self._arm_state(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + WB
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, xyz_w, quat, seed=None, at_tcp=True):
        """IK for a WORLD pose. If at_tcp, xyz is the fingertip (TCP) point."""
        xyz = np.array(xyz_w, dtype=float) - WB
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._arm_state(seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print('IK failed', None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        for i, (lo, hi) in enumerate(LIMITS):
            if not (lo <= q[i] <= hi):
                print('IK solution violates limit', JOINTS[i], q[i])
                return None
        return q

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f'move done code={code} max_joint_err={err:.4f}')
        return code, err

    def move_world(self, xyz_w, quat, seconds=3.0, seed=None):
        q = self.ik_world(xyz_w, quat, seed=seed)
        if q is None:
            return None
        self.move_q(q, seconds)
        p, _ = self.fk_hand()
        R = quat_R(*quat)
        tcp = p + TCP * R[:, 2]
        print('TCP now (world):', tcp.round(4))
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        print(f'gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}')
        return r


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
OPENRUA_EOF

# openrua op 16
timeout 300 python3 -u -c "
from ctl import *
r=Robot()
print('q',np.round(r.arm_q(),3))
p,q=r.fk_hand(); print('hand world',p.round(4),'quat',np.round(q,3))
R=quat_R(*q); print('hand z axis',R[:,2].round(3),'hand y axis',R[:,1].round(3))
print('TCP',(p+TCP*R[:,2]).round(4))
print('gap',r.finger_gap())
# test IK to a pose above the tomato can
print('IK above can', r.ik_world((-0.102,0.043,0.60),Q_DOWN))
print('IK above cc', r.ik_world((0.105,-0.19,0.60),Q_DOWN))
print('IK at cc', r.ik_world((0.105,-0.19,0.435),Q_DOWN))
print('IK above basket', r.ik_world((0.0,0.26,0.72),Q_DOWN))
"

# openrua op 17
timeout 300 python3 -u -c "
from ctl import *
r=Robot()
import copy
def tryik(xyz,quat,**kw):
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'
    req.ik_request.pose_stamped.header.frame_id = kw.get('frame','')
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    if kw.get('seed',True): req.ik_request.robot_state.joint_state = r._arm_state(r.arm_q())
    if kw.get('link'): req.ik_request.ik_link_name=kw['link']
    if 'timeout' in kw: req.ik_request.timeout.sec=kw['timeout']
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res=fut.result()
    print(xyz, kw, '->', res.error_code.val, np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)).get(j,0) for j in JOINTS],3) if res.error_code.val==1 else '')
# the current hand pose in base frame should be solvable
p,q=r.fk_hand(); pb=p-WB
tryik(pb,q)
tryik(pb,q,timeout=1)
tryik(pb,q,link='panda_hand')
tryik((0.4,0.0,0.4),(1,0,0,0))
tryik((0.4,0.0,0.4),(1,0,0,0),link='panda_hand')
tryik((0.4,0.0,0.4),(1,0,0,0),link='panda_link8')
tryik((0.4,0.0,0.4),(0.924,-0.383,0,0))
tryik((0.4,0.0,0.4),(0.924,-0.383,0,0),link='panda_link8')
tryik((0.4,0.0,0.4),(0.924,0.383,0,0),link='panda_link8')
"

# openrua op 18
timeout 300 python3 -u -c "
from ctl import *
r=Robot()
def tryik(xyz,quat,seed,link='panda_hand',avoid=None):
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'
    req.ik_request.ik_link_name=link
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    req.ik_request.robot_state.joint_state = r._arm_state(seed)
    if avoid is not None: req.ik_request.avoid_collisions=avoid
    req.ik_request.timeout.sec=1
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res=fut.result()
    print(xyz, seed, '->', res.error_code.val, np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)).get(j,0) for j in JOINTS],3) if res.error_code.val==1 else '')
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
print('FK ready', r.fk_hand(ready))
tryik((0.4,0.0,0.4),(1,0,0,0),ready)
tryik((0.4,0.0,0.4),(1,0,0,0),ready,avoid=False)
tryik((0.3,0.0,0.5),(1,0,0,0),ready)
tryik((0.3,0.0,0.5),(1,0,0,0),[0,0,0,-1.57,0,1.57,0.785])
"

# openrua op 19
timeout 300 python3 -u -c "
from ctl import *
r=Robot()
def tryik(xyz,quat,seed,link='panda_hand'):
    req = GetPositionIK.Request()
    req.ik_request.group_name = 'panda_arm'
    req.ik_request.ik_link_name=link
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    req.ik_request.robot_state.joint_state = r._arm_state(seed)
    req.ik_request.timeout.sec=1
    fut = r.ik.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res=fut.result()
    print(xyz, '->', res.error_code.val, np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)).get(j,0) for j in JOINTS],3) if res.error_code.val==1 else '')
cur=r.arm_q()
tryik((0.4-0.51,0.0,0.4+0.42),(1,0,0,0),cur)
tryik((-0.102,0.043,0.70),(1,0,0,0),cur)
tryik((0.105,-0.19,0.70),(1,0,0,0),cur)
tryik((0.0,0.26,0.80),(1,0,0,0),cur)
"

# openrua op 20
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace("WB = np.array([-0.51, 0.0, 0.42])","# verified empirically: FK/IK poses (frame_id empty) are already in WORLD\n# (the model root sits at world (-0.51, 0, 0.42)); no offset to apply.\nWB = np.array([0.0, 0.0, 0.0])")
s=s.replace("        req.ik_request.group_name = M['planning']['group']\n","        req.ik_request.group_name = M['planning']['group']\n        req.ik_request.ik_link_name = 'panda_hand'\n")
s=s.replace("        req.ik_request.avoid_collisions = False\n        req.ik_request.timeout.sec = 2\n","        req.ik_request.timeout.sec = 1\n")
open('ctl.py','w').write(s)
EOF
grep -n "WB =\|ik_link_name\|timeout.sec" ctl.py

# openrua op 21
mkdir -p "$(dirname /workspace/pick1.py)"
cat > /workspace/pick1.py <<'OPENRUA_EOF'
from ctl import *
r = Robot()
CAN = np.array([-0.102, 0.043])
print('open gripper'); r.gripper(0.04)
print('pre-grasp'); q1 = r.move_world((CAN[0], CAN[1], 0.65), Q_DOWN, 4.0)
if q1 is None: sys.exit(1)
print('descend 1'); q2 = r.move_world((CAN[0], CAN[1], 0.55), Q_DOWN, 2.0)
if q2 is None: sys.exit(1)
print('descend 2'); q3 = r.move_world((CAN[0], CAN[1], 0.47), Q_DOWN, 2.0)
if q3 is None: sys.exit(1)
print('close'); r.gripper(0.0)
print('gap', r.finger_gap())
print('lift'); r.move_world((CAN[0], CAN[1], 0.70), Q_DOWN, 3.0)
print('gap after lift', r.finger_gap())
OPENRUA_EOF

# openrua op 22
nohup python3 -u pick1.py > pick1.log 2>&1 &
sleep 45; cat /workspace/pick1.log

# openrua op 23
sleep 15; cat /workspace/pick1.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
mkdir -p "$(dirname /workspace/place1.py)"
cat > /workspace/place1.py <<'OPENRUA_EOF'
from ctl import *
r = Robot()
B = np.array([-0.003, 0.26])
print('gap', r.finger_gap())
print('to basket high'); q = r.move_world((B[0], B[1], 0.76), Q_DOWN, 4.0)
if q is None: sys.exit(1)
print('gap', r.finger_gap())
print('lower'); r.move_world((B[0], B[1], 0.72), Q_DOWN, 2.0)
print('release'); r.gripper(0.04)
print('retreat'); r.move_world((B[0], B[1], 0.85), Q_DOWN, 3.0)
OPENRUA_EOF

# openrua op 25
nohup python3 -u place1.py > place1.log 2>&1 &
sleep 50; cat /workspace/place1.log

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview

# openrua op 27
timeout 120 python3 pc.py robot0_robotview && python3 -c "
import numpy as np, cv2
Wp=np.load('robot0_robotview_world.npy')
z=Wp[...,2]
m=(z>0.437)&(z<0.50)&(Wp[...,0]>0.05)&(Wp[...,0]<0.2)&(Wp[...,1]<-0.12)&(Wp[...,1]>-0.30)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<100: continue
    pts=Wp[lab==i]
    print(i,stats[i,4],cent[i].round(0),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    top=pts[pts[:,2]>pts[:,2].max()-0.01]
    print('  top face mean',top.mean(0).round(3),'n',len(top))
    xy=top[:,:2]-top[:,:2].mean(0)
    w,v=np.linalg.eigh(xy.T@xy)
    print('  long axis',v[:,1].round(3),'yaw deg',np.degrees(np.arctan2(v[1,1],v[0,1])).round(1))
"

# openrua op 28
mkdir -p "$(dirname /workspace/pick2.py)"
cat > /workspace/pick2.py <<'OPENRUA_EOF'
from ctl import *
r = Robot()
CC = np.array([0.10, -0.19])
B = np.array([-0.003, 0.29])
print('gap', r.finger_gap())
print('pre-grasp'); q1 = r.move_world((CC[0], CC[1], 0.65), Q_DOWN, 4.0)
if q1 is None: sys.exit(1)
print('descend 1'); q2 = r.move_world((CC[0], CC[1], 0.52), Q_DOWN, 2.0)
if q2 is None: sys.exit(1)
print('descend 2'); q3 = r.move_world((CC[0], CC[1], 0.436), Q_DOWN, 2.0)
if q3 is None: sys.exit(1)
print('close'); r.gripper(0.0)
print('gap', r.finger_gap())
print('lift'); r.move_world((CC[0], CC[1], 0.70), Q_DOWN, 3.0)
g = r.finger_gap(); print('gap after lift', g)
if g < 0.02:
    print('GRASP FAILED'); sys.exit(2)
print('to basket high'); q = r.move_world((B[0], B[1], 0.78), Q_DOWN, 4.0)
if q is None: sys.exit(1)
print('gap', r.finger_gap())
print('lower'); r.move_world((B[0], B[1], 0.71), Q_DOWN, 2.0)
print('release'); r.gripper(0.04)
print('retreat'); r.move_world((B[0], B[1], 0.85), Q_DOWN, 3.0)
OPENRUA_EOF

# openrua op 29
nohup python3 -u pick2.py > pick2.log 2>&1 &
sleep 60; cat /workspace/pick2.log

# openrua op 30
sleep 40; cat /workspace/pick2.log; pgrep -f pick2.py || echo finished

# openrua op 31
sleep 3; pgrep -f pick2.py || echo finished; timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_robotview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 32
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_basket.png', cv2.resize(im[250:330,340:430],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png')
cv2.imwrite('crop_basket2.png', cv2.resize(im[200:320,440:620],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 33
timeout 200 python3 -u -c "
from ctl import *
r=Robot()
r.move_world((-0.2,-0.25,0.80),Q_DOWN,4.0)
" && timeout 60 python3 tools/perception/cam_snap.py birdview && timeout 60 python3 tools/perception/cam_snap.py galleryview && timeout 60 python3 tools/perception/cam_snap.py paperview && timeout 120 python3 pc.py birdview

# openrua op 34
python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_basket.png', cv2.resize(im[250:330,340:430],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"; python3 -c "
import numpy as np
Wp=np.load('birdview_world.npy')
# points within basket interior footprint
m=(Wp[...,0]>-0.06)&(Wp[...,0]<0.055)&(Wp[...,1]>0.195)&(Wp[...,1]<0.325)&np.isfinite(Wp[...,2])
pts=Wp[m]; print('interior pts',len(pts),'z range',pts[:,2].min().round(3),pts[:,2].max().round(3))
hist,edges=np.histogram(pts[:,2],bins=np.arange(0.40,0.70,0.02)); print(list(zip(edges[:-1].round(2),hist)))
# cream cheese original location and can location: anything above table?
for name,(x,y) in {'cc_orig':(0.10,-0.19),'can_orig':(-0.102,0.043)}.items():
    m=(np.abs(Wp[...,0]-x)<0.04)&(np.abs(Wp[...,1]-y)<0.03)&np.isfinite(Wp[...,2])
    print(name,'max z',Wp[m][:,2].max().round(3))
"
