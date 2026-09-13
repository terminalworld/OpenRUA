#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 python3 - <<'EOF'
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
import rclpy.time
rclpy.init(); node=rclpy.create_node('tfprobe')
buf=Buffer(); TransformListener(buf,node)
import time
for _ in range(20): rclpy.spin_once(node,timeout_sec=0.2)
print(buf.all_frames_as_string())
for a,b in [('world','panda_link0'),('world','panda_hand'),('world','birdview_optical_frame'),('world','agentview_optical_frame'),('panda_link0','panda_hand')]:
    try:
        t=buf.lookup_transform(a,b,rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(a,'->',b, round(tr.x,4),round(tr.y,4),round(tr.z,4),'q',round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
    except Exception as e: print(a,'->',b,'ERR',e)
EOF

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 20 ros2 topic echo /birdview/color/camera_info --once | head -30

# openrua op 7
timeout 60 python3 - <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
fx=fy=579.4112549695428; cx=320; cy=240
# camera at world (-0.2,0,3.0), q=(0.7071,0.7071,0,0): rotation maps optical -> world
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
Rm=R(0.7071,0.7071,0,0); t=np.array([-0.2,0,3.0])
def w(u,v):
    z=d[v,u]; p=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z]); return Rm@p+t
# table height: sample an empty area
for (u,v) in [(400,350),(320,265),(322,255),(322,275),(360,240),(260,185),(300,175),(230,175),(285,175),(560,300)]:
    print((u,v), d[v,u], np.round(w(u,v),3))
# height map
H=np.zeros_like(d)
for v in range(0,480,1):
    for u in range(0,640,1):
        H[v,u]=w(u,v)[2]
np.save('bird_H.npy',H)
table=np.median(H[300:340,380:440]); print('table z',table)
mask=H>table+0.01
ys,xs=np.where(mask[100:330,150:450]); 
print('elevated px count',len(xs))
EOF

# openrua op 8
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
H=np.load('bird_H.npy'); d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
table=0.8804
mask=((H>table+0.015)&(H<1.3)).astype(np.uint8)
# limit to table region rows 100-330
mask[:100]=0; mask[330:]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a<20: continue
    ys,xs=np.where(lab==i)
    zs=H[ys,xs]
    # world coords of bbox corners
    def wxy(u,v,z=None):
        zz=d[v,u] if z is None else z
        return ((v-cy)*zz/fx - 0.2 + 0.0, (u-cx)*zz/fx)  # approx: X_world = -0.2 + Y_opt ; Y_world = X_opt
    print(f'comp {i}: bbox u[{x},{x+w}] v[{y},{y+h}] area {a} zmax {zs.max():.3f} zmed {np.median(zs):.3f} cent px {cent[i].round(1)}')
    # world extents using top z
    ztop=np.median(zs); zc=3.0-ztop
    print('   world x range', round((y-cy)*zc/fx-0.2,3), round((y+h-cy)*zc/fx-0.2,3), ' y range', round((x-cx)*zc/fx,3), round((x+w-cx)*zc/fx,3))
EOF

# openrua op 9
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
img=cv2.imread('birdview.png'); crop=img[140:300,190:400]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
H=np.load('bird_H.npy')
c=H[150:230,205:315]
# print height in cm above table on a grid every 4 px
np.set_printoptions(linewidth=250)
for v in range(150,230,3):
    print(v, ' '.join(f'{int(round((H[v,u]-0.88)*100)):3d}' for u in range(205,315,4)))
EOF

# openrua op 10
timeout 60 python3 - <<'EOF'
import cv2
img=cv2.imread('agentview.png'); cv2.imwrite('agent_crop.png', cv2.resize(img[140:300,100:380],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('eih_crop.png', cv2.resize(img[260:420,200:640],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 11
timeout 30 python3 - <<'EOF'
import rclpy, numpy as np
from tf2_ros import Buffer, TransformListener
import rclpy.time
rclpy.init(); node=rclpy.create_node('tfprobe2')
buf=Buffer(); TransformListener(buf,node)
for _ in range(15): rclpy.spin_once(node,timeout_sec=0.2)
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for f in ['frontview','sideview','agentview','robot0_robotview','robot0_eye_in_hand','birdview','galleryview','paperview']:
    t=buf.lookup_transform('world',f+'_optical_frame',rclpy.time.Time())
    tr=t.transform.translation; q=t.transform.rotation
    Rm=R(q.x,q.y,q.z,q.w)
    print(f, 'pos',round(tr.x,3),round(tr.y,3),round(tr.z,3),'viewdir(+Zopt)',Rm[:,2].round(2),'img-right(+Xopt)',Rm[:,0].round(2))
EOF

# openrua op 12
timeout 600 python3 tools/action/fjt_send.py 1.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854 3 && timeout 20 ros2 topic echo /joint_states --once | head -22 && timeout 60 python3 tools/perception/cam_snap.py birdview birdview2.png && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth2.png

# openrua op 13
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
d=np.load('birdview_depth2.npy')
fx=579.4112549695428; cx=320; cy=240
# world: x = -0.2 + (v-cy)*z/fx ; y = (u-cx)*z/fx ; Z = 3.0 - z
V,U=np.mgrid[0:480,0:640]
X=-0.2+(V-cy)*d/fx; Y=(U-cx)*d/fx; Z=3.0-d
np.save('bird2_XYZ.npy',np.stack([X,Y,Z]))
img=cv2.imread('birdview2.png'); cv2.imwrite('bird2_crop.png', cv2.resize(img[150:225,200:350],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
np.set_printoptions(linewidth=250)
print('cols u:', ' '.join(f'{u:3d}' for u in range(205,345,4)))
for v in range(156,216,2):
    print(v, ' '.join(f'{int(round((Z[v,u]-0.88)*100)):3d}' for u in range(205,345,4)))
EOF

# openrua op 14
timeout 60 python3 - <<'EOF'
import numpy as np
X,Y,Z=np.load('bird2_XYZ.npy')
np.set_printoptions(linewidth=250)
print('cols u:', ' '.join(f'{u:3d}' for u in range(308,336,1)))
for v in range(244,290,1):
    print(v, ' '.join(f'{int(round((Z[v,u]-0.88)*100)):3d}' for u in range(308,336,1)))
# book point cloud
m=(Z>0.90)&(Z<1.2); m[:230]=0; m[300:]=0; m[:, :300]=0; m[:,340:]=0
xs,ys,zs=X[m],Y[m],Z[m]
print('book pts',m.sum(),'x',xs.min(),xs.max(),'y',ys.min(),ys.max(),'z',zs.min(),zs.max())
# top points
t=zs>zs.max()-0.02
print('top x',xs[t].min(),xs[t].max(),'y',ys[t].min(),ys[t].max(),'mean',xs[t].mean(),ys[t].mean())
EOF

# openrua op 15
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Helper library for this Panda workstation: state, IK, trajectory, gripper, cameras."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import rclpy.time

M = yaml.safe_load(open('/workspace/machine.yaml'))
FJT = next(a for a in M['actuators'] if a['kind'] == 'joint_trajectory')
GRIP = next(a for a in M['actuators'] if a['kind'] == 'gripper')
JOINTS = FJT['joints']
BASE = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0
TCP = M['hand']['tcp_offset_m']


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1; x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def down_quat(yaw):
    """Gripper pointing straight down, fingers closing along world direction rotated by yaw from +y.
    yaw=0 -> fingers close along world y; yaw=pi/2 -> along world x."""
    qz = np.array([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)])
    return quat_mul(qz, np.array([1.0, 0, 0, 0]))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('rob_helper')
        self.js = {}
        self.node.create_subscription(JointState, '/joint_states', lambda m: self.js.__setitem__('m', m), 1)
        self.wr = {}
        self.node.create_subscription(WrenchStamped, '/franka_robot_state_broadcaster/external_wrench',
                                      lambda m: self.wr.__setitem__('m', m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT['port'])
        self.grip = ActionClient(self.node, GripperCommand, GRIP['port'])
        self.ik = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk = self.node.create_client(GetPositionFK, '/compute_fk')
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.pop('m', None)
        while 'm' not in self.js:
            self.spin(0.2)
        m = self.js['m']
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j['panda_finger_joint1'], j['panda_finger_joint2']

    def wrench(self):
        self.wr.pop('m', None)
        while 'm' not in self.wr:
            self.spin(0.2)
        w = self.wr['m'].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def hand_pose_world(self):
        """FK of panda_hand in world frame -> (pos, quat)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        seed = JointState()
        j = self.joints()
        for n in JOINTS:
            seed.name.append(n); seed.position.append(j[n])
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        return pos + TCP * quat_to_R(*q)[:, 2], q

    def solve_ik(self, tcp_world, quat, seed=None, tcp=True):
        """IK for a TCP (or hand) pose in world. Returns joint list or None."""
        quat = np.asarray(quat, float)
        p = np.asarray(tcp_world, float)
        if tcp:
            p = p - TCP * quat_to_R(*quat)[:, 2]
        p = p - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M['planning']['group']
        req.ik_request.pose_stamped.header.frame_id = ''
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        s = JointState()
        seedq = seed if seed is not None else self.arm_q()
        for n, v in zip(JOINTS, seedq):
            s.name.append(n); s.position.append(float(v))
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print('IK failed', None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def move_joints(self, targets, seconds=3.0, tol=0.01, retries=2):
        """Send a joint trajectory (list of (positions, t) or one positions list). Verify."""
        if not isinstance(targets[0], (list, tuple, np.ndarray)):
            targets = [(targets, seconds)]
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            for pos, t in targets:
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                goal.trajectory.points.append(pt)
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            q = np.array(self.arm_q()); err = np.abs(q - np.array(targets[-1][0])).max()
            print(f'  traj code={code} max joint err={err:.4f}')
            if err < tol:
                return True
            targets = [(targets[-1][0], max(2.0, targets[-1][1] / 2))]
        return err < tol

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.solve_ik(tcp_world, quat, seed)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        p, _ = self.tcp_world()
        print('  tcp now', p.round(4), 'target', np.round(tcp_world, 4))
        return ok

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP['max_effort'])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f'  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}')
        return f

    def grab(self, topic, msg_type, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault('m', m), 1)
        t0 = time.time()
        while 'm' not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get('m')

    def snap(self, cam, out=None):
        m = self.grab(f'/{cam}/color/image_raw', Image)
        img = self.bridge.imgmsg_to_cv2(m, 'bgr8')
        if out:
            import cv2; cv2.imwrite(out, img)
        return img

    def depth(self, cam):
        m = self.grab(f'/{cam}/depth/image_raw', Image)
        return self.bridge.imgmsg_to_cv2(m, 'passthrough').astype(np.float32)

    def cam_info(self, cam):
        m = self.grab(f'/{cam}/color/camera_info', CameraInfo)
        return np.array(m.k).reshape(3, 3)

    def cam_tf(self, cam):
        frame = f'{cam}_optical_frame'
        for _ in range(50):
            self.spin(0.1)
            if self.tfbuf.can_transform('world', frame, rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform('world', frame, rclpy.time.Time())
        tr = t.transform.translation; q = t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), quat_to_R(q.x, q.y, q.z, q.w)

    def cloud(self, cam, cam_pos=None, cam_R=None):
        """Depth image -> world XYZ arrays (H,W,3)."""
        d = self.depth(cam); K = self.cam_info(cam)
        if cam_pos is None:
            cam_pos, cam_R = self.cam_tf(cam)
        H, W = d.shape
        v, u = np.mgrid[0:H, 0:W]
        x = (u - K[0, 2]) * d / K[0, 0]; y = (v - K[1, 2]) * d / K[1, 1]
        P = np.stack([x, y, d], -1) @ cam_R.T + cam_pos
        return P, d
OPENRUA_EOF

# openrua op 16
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
print('joints', np.round(r.arm_q(),3), 'fingers', r.fingers())
p,q=r.tcp_world(); print('tcp world', p.round(4), q.round(4))
hp,hq=r.hand_pose_world(); print('hand world', hp.round(4))
print('wrench', r.wrench().round(2))
# IK test: above the book
sol=r.solve_ik([-0.114,0.003,1.10], down_quat(0.0))
print('IK sol', None if sol is None else np.round(sol,3))
EOF

# openrua op 17
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
req = GetPositionFK.Request()
req.fk_link_names = ['panda_link0','panda_hand']
seed = JointState(); j = r.joints()
for n in JOINTS: seed.name.append(n); seed.position.append(j[n])
req.robot_state.joint_state = seed
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res=fut.result()
for ps in res.pose_stamped:
    print(ps.header.frame_id, ps.pose.position, ps.pose.orientation)
print(res.error_code)
# TF hand
for _ in range(20): r.spin(0.1)
t=r.tfbuf.lookup_transform('world','panda_hand',rclpy.time.Time()); print('TF world->hand', t.transform.translation)
t=r.tfbuf.lookup_transform('panda_link0','panda_hand',rclpy.time.Time()); print('TF link0->hand', t.transform.translation)
EOF

# openrua op 18
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
hand_world=np.array([-0.37876,0.26655,1.26962]); q=[0.95181,0.30536,-0.02706,0.00864]
cur=r.arm_q()
for label,p in [('world',hand_world),('base',hand_world-BASE)]:
    # bypass BASE subtraction: pass p+BASE so solve_ik subtracts it back
    sol=r.solve_ik(p+BASE, q, tcp=False)
    print(label, None if sol is None else np.round(np.array(sol)-np.array(cur),3))
EOF

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.75, 0.0, 0.912])  # world -> panda_link0", "new_string": "BASE = np.array([0.0, 0.0, 0.0])  # verified: MoveIt FK/IK here operate in WORLD coords (link0 at -0.75,0,0.912)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
p,q=r.tcp_world(); print('tcp world', p.round(4), q.round(4))
cur=r.arm_q()
for z in [1.10,1.15,1.20]:
    sol=r.solve_ik([-0.114,0.003,z], down_quat(0.0))
    print(z, None if sol is None else np.round(sol,3))
EOF

# openrua op 21
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
ok=r.move_tcp([-0.114,0.003,1.15], down_quat(0.0), 4.0)
print('ok',ok)
p,q=r.tcp_world(); print('tcp', p.round(4), 'quat', q.round(4), 'R z', quat_to_R(*q)[:,2].round(3), 'R y', quat_to_R(*q)[:,1].round(3))
r.snap('robot0_eye_in_hand','eih1.png')
EOF

# openrua op 22
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
req = GetPositionFK.Request()
req.fk_link_names = ['panda_link8','panda_hand','panda_leftfinger','panda_rightfinger']
seed = JointState(); j = r.joints()
for n in JOINTS: seed.name.append(n); seed.position.append(j[n])
req.robot_state.joint_state = seed
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n,ps in zip(req.fk_link_names,fut.result().pose_stamped):
    p=ps.pose.position; q=ps.pose.orientation
    print(n, round(p.x,4),round(p.y,4),round(p.z,4),'q',round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
EOF
timeout 30 ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -A3 -i 'group name="panda_arm"' | head; timeout 30 ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -i -E 'chain|end_effector|tip' | head

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        ps.position.x, ps.position.y, ps.position.z = map(float, p)\\n        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)", "new_string": "        ps.position.x, ps.position.y, ps.position.z = map(float, p)\\n        # IK tip link is panda_link8; panda_hand = link8 rotated -45deg about z\\n        # (same origin). Convert the requested HAND quaternion to link8.\\n        q8 = quat_mul(quat, np.array([0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)]))\\n        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q8)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
ok=r.move_tcp([-0.114,0.003,1.15], down_quat(0.0), 3.0)
p,q=r.tcp_world(); R=quat_to_R(*q); print('tcp', p.round(4), 'hand y', R[:,1].round(3), 'hand z', R[:,2].round(3))
r.snap('robot0_eye_in_hand','eih2.png')
P,d=r.cloud('robot0_eye_in_hand')
np.save('eih2_P.npy',P)
cp,cR=r.cam_tf('robot0_eye_in_hand'); print('cam pos', cp.round(4), 'cam z', cR[:,2].round(3))
Z=P[...,2]
m=(Z>0.95)&(Z<1.1)&np.isfinite(Z)
pts=P[m]; print('book pts', len(pts))
if len(pts):
    print('x',pts[:,0].min().round(4),pts[:,0].max().round(4),'y',pts[:,1].min().round(4),pts[:,1].max().round(4),'z',pts[:,2].min().round(4),pts[:,2].max().round(4))
    top=pts[pts[:,2]>pts[:,2].max()-0.015]
    print('top n',len(top),'center',top[:,:2].mean(0).round(4),'x',top[:,0].min().round(4),top[:,0].max().round(4),'y',top[:,1].min().round(4),top[:,1].max().round(4))
    # PCA orientation of top
    c=top[:,:2]-top[:,:2].mean(0); w,v=np.linalg.eigh(c.T@c); print('long axis', v[:,1].round(3))
EOF

# openrua op 25
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
yaw=-0.075
q=down_quat(yaw)
print('fingers before', r.fingers())
ok=r.move_tcp([-0.1109,0.0034,1.05], q, 2.5); 
ok=r.move_tcp([-0.1109,0.0034,0.985], q, 2.5)
p,qq=r.tcp_world(); print('tcp', p.round(4))
print('wrench', r.wrench().round(2))
r.snap('robot0_eye_in_hand','eih3.png'); r.snap('agentview','agent3.png')
EOF

# openrua op 26
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
f=r.gripper(0.01)
for _ in range(3):
    r.spin(0.3); print('fingers', np.round(r.fingers(),4))
print('wrench', r.wrench().round(2))
EOF

# openrua op 27
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(-0.075)
ok=r.move_tcp([-0.1109,0.0034,1.10], q, 2.5)
ok=r.move_tcp([-0.1109,0.0034,1.25], q, 2.5)
print('fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(2))
r.snap('agentview','agent4.png'); r.snap('frontview','front4.png')
EOF

# openrua op 28
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
cur=r.arm_q(); print('j7 now', round(cur[6],3))
for yaw in [np.pi/2, -np.pi/2]:
    sol=r.solve_ik([-0.4275,-0.154,1.25], down_quat(yaw))
    print(yaw, None if sol is None else np.round(sol,3))
EOF

# openrua op 29
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
ok=r.move_tcp([-0.4275,-0.154,1.25], q, 4.0)
p,qq=r.tcp_world(); R=quat_to_R(*qq); print('tcp', p.round(4), 'hand y', R[:,1].round(3))
print('fingers', np.round(r.fingers(),4))
r.snap('robot0_eye_in_hand','eih5.png'); r.snap('agentview','agent5.png'); r.snap('birdview','bird5.png')
EOF

# openrua op 30
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
sol=r.solve_ik([-0.4275,-0.154,1.25], q)
print('sol', np.round(sol,3), 'cur', np.round(r.arm_q(),3))
ok=r.move_joints(sol, 6.0, retries=2)
p,qq=r.tcp_world(); R=quat_to_R(*qq); print('tcp', p.round(4), 'hand y', R[:,1].round(3), 'hand z', R[:,2].round(3))
print('fingers', np.round(r.fingers(),4))
r.snap('robot0_eye_in_hand','eih5.png'); r.snap('birdview','bird5.png'); r.snap('frontview','front5.png'); r.snap('sideview','side5.png')
EOF

# openrua op 31
timeout 300 python3 -u - <<'EOF'
from rob import *
r=Robot()
P,d=r.cloud('robot0_eye_in_hand'); np.save('eih5_P.npy',P)
cp,cR=r.cam_tf('robot0_eye_in_hand'); print('cam', cp.round(4))
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
# wall tops: z in [1.03,1.08], within the caddy x range
m=(Z>1.03)&(Z<1.08)&(X>-0.5)&(X<-0.3)&np.isfinite(Z)
pts=P[m]; print('walltop pts', len(pts))
# histogram of y for wall-top points
h,e=np.histogram(pts[:,1],bins=np.arange(-0.40,0.10,0.005))
for c,lo in zip(h,e[:-1]):
    if c>0: print(f'y {lo:.3f} {c}')
print('--- x histogram of wall tops')
h,e=np.histogram(pts[:,0],bins=np.arange(-0.50,-0.30,0.005))
for c,lo in zip(h,e[:-1]):
    if c>0: print(f'x {lo:.3f} {c}')
# low divider z~0.99 (11cm above table)
m2=(Z>0.96)&(Z<1.02)&(X>-0.5)&(X<-0.3)&(Y<-0.09)&(Y>-0.22)&np.isfinite(Z)
p2=P[m2]; print('lowdiv pts', len(p2))
if len(p2): print('x', np.percentile(p2[:,0],[5,50,95]).round(4), 'y', np.percentile(p2[:,1],[5,50,95]).round(4), 'z', np.percentile(p2[:,2],[5,50,95]).round(4))
# book: pts z>1.2
m3=(Z>1.15)&np.isfinite(Z)&(X>-0.5)&(X<-0.3)
p3=P[m3]; print('book pts', len(p3))
if len(p3): print('x', p3[:,0].min().round(4), p3[:,0].max().round(4), 'y', p3[:,1].min().round(4), p3[:,1].max().round(4), 'z', p3[:,2].min().round(4), p3[:,2].max().round(4))
EOF

# openrua op 32
timeout 300 python3 -u - <<'EOF'
import numpy as np
P=np.load('eih5_P.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(Z)
def hist(m,axis,lo,hi,step=0.0025,label=''):
    pts=P[m&ok]; h,e=np.histogram(pts[:,axis],bins=np.arange(lo,hi,step))
    print(label, 'n',len(pts)); print('  '+' '.join(f'{l:.4f}:{c}' for c,l in zip(h,e[:-1]) if c>0))
# floor of middle compartment (z 0.89-0.92), x in [-0.45,-0.34]
hist((Z>0.885)&(Z<0.925)&(X>-0.45)&(X<-0.34)&(Y>-0.25)&(Y<-0.05),1,-0.25,-0.05,label='floor y')
# wall tops z>1.03 in x range of middle compartment interior (-0.45..-0.34)
hist((Z>1.03)&(Z<1.08)&(X>-0.45)&(X<-0.34)&(Y>-0.25)&(Y<-0.05),1,-0.25,-0.05,label='walltop y (x -0.45..-0.34)')
# cross divider z 0.98-1.02
hist((Z>0.98)&(Z<1.02)&(X>-0.45)&(X<-0.34)&(Y>-0.25)&(Y<-0.05),0,-0.45,-0.34,label='crossdiv x')
hist((Z>0.98)&(Z<1.02)&(X>-0.45)&(X<-0.34)&(Y>-0.25)&(Y<-0.05),1,-0.25,-0.05,label='crossdiv y')
# back wall top x
hist((Z>1.03)&(Z<1.08)&(Y>-0.20)&(Y<-0.10),0,-0.50,-0.30,label='walltop x (y -0.2..-0.1)')
# floor x range in back compartment
hist((Z>0.885)&(Z<0.925)&(Y>-0.20)&(Y<-0.10),0,-0.50,-0.30,label='floor x (y -0.2..-0.1)')
# book: z between 1.26 and 1.30 (top pages), exclude fingers
hist((Z>1.26)&(Z<1.32),1,-0.25,-0.05,label='book top y')
hist((Z>1.26)&(Z<1.32),0,-0.50,-0.30,label='book top x')
EOF

# openrua op 33
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
w0=r.wrench(); print('wrench0', w0.round(2))
for z in [1.18, 1.12]:
    ok=r.move_tcp([-0.4277,-0.1525,z], q, 2.5)
    print('  wrench', r.wrench().round(2), 'fingers', np.round(r.fingers(),4))
r.snap('agentview','agent6.png'); r.snap('frontview','front6.png')
EOF

# openrua op 34
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
for z in [1.06, 1.02]:
    ok=r.move_tcp([-0.4277,-0.1525,z], q, 2.5)
    print('  wrench', r.wrench().round(2), 'fingers', np.round(r.fingers(),4))
r.snap('agentview','agent7.png'); r.snap('frontview','front7.png'); r.snap('sideview','side7.png')
EOF

# openrua op 35
timeout 60 python3 - <<'EOF'
import cv2
img=cv2.imread('side7.png'); cv2.imwrite('side7_crop.png', cv2.resize(img[150:350,330:530],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
img=cv2.imread('agent7.png'); cv2.imwrite('agent7_crop.png', cv2.resize(img[100:300,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 36
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
p,_=r.tcp_world(); print('tcp', p.round(4))
ok=r.move_tcp([p[0],p[1],1.075], q, 2.0)
print('  wrench', r.wrench().round(2), 'fingers', np.round(r.fingers(),4))
r.snap('frontview','front8.png')
EOF

# openrua op 37
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
ok=r.move_tcp([-0.427,-0.1525,1.075], q, 2.0)
print('  wrench', r.wrench().round(2), 'fingers', np.round(r.fingers(),4))
r.snap('frontview','front9.png'); r.snap('agentview','agent9.png')
EOF

# openrua op 38
timeout 60 python3 - <<'EOF'
import cv2
img=cv2.imread('agent9.png'); cv2.imwrite('agent9_crop.png', cv2.resize(img[100:300,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 39
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(np.pi/2)
f=r.gripper(0.04)
print('wrench', r.wrench().round(2))
ok=r.move_tcp([-0.427,-0.1525,1.25], q, 2.5)
print('fingers', np.round(r.fingers(),4), 'wrench', r.wrench().round(2))
r.snap('agentview','agent10.png'); r.snap('frontview','front10.png'); r.snap('robot0_eye_in_hand','eih10.png')
EOF

# openrua op 40
timeout 60 python3 - <<'EOF'
import cv2
img=cv2.imread('agent10.png'); cv2.imwrite('agent10_crop.png', cv2.resize(img[100:300,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 41
timeout 900 python3 -u - <<'EOF'
from rob import *
r=Robot()
q=down_quat(0.0)
ok=r.move_tcp([-0.25,0.30,1.25], q, 4.0)
P,d=r.cloud('birdview'); np.save('bird_final_P.npy',P)
r.snap('birdview','bird_final.png'); r.snap('agentview','agent_final.png'); r.snap('frontview','front_final.png')
X,Y,Z=P[...,0],P[...,1],P[...,2]
# objects above the caddy floor within the caddy footprint, below wall tops
m=(Z>0.93)&(Z<1.05)&(X>-0.45)&(X<-0.34)&(Y>-0.21)&(Y<-0.09)&np.isfinite(Z)
pts=P[m]; print('pts in middle compartment above floor', len(pts))
if len(pts):
    print('x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].min().round(3),pts[:,2].max().round(3))
    top=pts[pts[:,2]>pts[:,2].max()-0.02]
    print('top center', top[:,:2].mean(0).round(4), 'ztop', pts[:,2].max().round(3))
# check the book is not elsewhere on the table: anything above table outside caddy/mug?
m2=(Z>0.93)&(Z<1.1)&np.isfinite(Z); m2[:100]=0; m2[330:]=0
pts2=P[m2]; print('all elevated pts: x range',pts2[:,0].min().round(3),pts2[:,0].max().round(3))
h,e=np.histogram(pts2[:,0],bins=np.arange(-0.5,0.3,0.02)); print(' '.join(f'{l:.2f}:{c}' for c,l in zip(h,e[:-1]) if c>0))
EOF

# openrua op 42
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified quirks of the /workspace Franka Panda ROS2/MoveIt sim workstation (IK frame, tip link, controller lag, gripper) that contradict or extend the starter docs
metadata:
  type: project
---

Verified on 2026-09-12 while doing the book-into-caddy task on the Panda workstation in /workspace:

- MoveIt `/compute_ik` and `/compute_fk` here work in WORLD coordinates (panda_link0 sits at world (-0.75, 0, 0.912)); the docs' "planner frame is the arm base" note did NOT apply — base-frame targets return -31 (no solution).
- IK group `panda_arm` tip link is `panda_link8`, which is rotated -45° about z relative to `panda_hand` (same origin). Pass hand_quat * rotz(+45°) to get the fingers where you want them.
- FollowJointTrajectory lags badly on big moves (joint7 especially); error_code -5 with the target partly reached is common. Use 5-6 s durations for large moves and resend to converge; 2-3 s is fine for short vertical moves.
- Gripper: fingers can shift asymmetrically under side load (e.g. 0.005/-0.0246) while still gripping; they recentre when the load is removed. Finger pair + a 3 cm object needs ~7 cm of clearance — fingers won't fit into a 5.4 cm slot; release from above the rim and let the object drop.
- Table top z=0.88; caddy floor 0.90, walls to 1.06. Eye-in-hand camera sits ~5 cm along hand +x from the TCP at the hand origin height; its depth at <10 cm range is mm-accurate and beats the birdview (3.4 mm/px).
- Helper library written at /workspace/rob.py (Robot class: IK/FK, move_tcp, gripper, clouds).

**Why:** these cost most of the session to discover and are not in machine.yaml or docs.
**How to apply:** trust rob.py conventions; verify with FK on `panda_hand` after the first move on any new session.
OPENRUA_EOF

# openrua op 43
cd /claude-config/projects/-workspace/memory && (cat MEMORY.md 2>/dev/null; echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK is world-frame, tip link panda_link8 (45° off hand), controller lag, gripper clearance") > MEMORY.md.tmp && mv MEMORY.md.tmp MEMORY.md && cat MEMORY.md
