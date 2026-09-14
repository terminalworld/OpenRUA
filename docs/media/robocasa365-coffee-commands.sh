#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0  ran 30.0s
ls -la /workspace

# openrua op 1  ran 2.6s
ls -la /workspace/docs /workspace/tools; find /workspace/tools -type f | head -50

# openrua op 2  ran 7.4s
timeout 60 ros2 topic list 2>&1; echo "=== ACTIONS ==="; timeout 60 ros2 action list 2>&1; echo "=== NODES ==="; timeout 60 ros2 node list 2>&1

# openrua op 3  ran 7.7s
timeout 60 ros2 topic echo /joint_states --once 2>&1 | head -40; echo "=== ODOM ==="; timeout 60 ros2 topic echo /odom --once 2>&1 | head -30

# openrua op 4  ran 8.2s
timeout 120 python3 tools/perception/cam_snap.py robot0_agentview_left agentview_left.png 2>&1 | tail -2; timeout 120 python3 tools/perception/cam_snap.py robot0_agentview_right agentview_right.png 2>&1 | tail -2; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eye_in_hand.png 2>&1 | tail -2; ls -la *.png

# openrua op 5  ran 21.7s
for px in "325 100" "325 110" "322 105" "330 108"; do echo -n "left $px -> "; timeout 90 python3 tools/perception/px2world.py robot0_agentview_left $px 2>&1 | grep -v IP_LOCATOR | tail -1; done

# openrua op 6  ran 1.9s
timeout 60 ros2 topic list | grep -i -E "camera_info" ; echo "=== TF frames ==="; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "child_frame_id|frame_id" | head -40

# openrua op 7  ran 7.0s
python3 -c "
import cv2
img = cv2.imread('agentview_left.png')
print(img.shape)
crop = img[60:150, 280:380]
cv2.imwrite('mug_crop.png', cv2.resize(crop, None, fx=6, fy=6, interpolation=cv2.INTER_NEAREST))
crop2 = img[140:320, 320:470]
cv2.imwrite('cm_crop.png', cv2.resize(crop2, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
"

# openrua op 8  ran 40.3s
timeout 60 python3 -u -c "
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfdump')
b=Buffer(); TransformListener(b,n)
for _ in range(80): rclpy.spin_once(n,timeout_sec=0.1)
print(b.all_frames_as_yaml())
" 2>&1 | grep -v IP_LOCATOR

# openrua op 9  ran 13.8s
timeout 60 ros2 service list 2>&1 | grep -v IP_LOCATOR | grep -v parameter | head -40; echo "=== w->link0 ==="; timeout 60 python3 -u -c "
import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('t')
b=Buffer(); TransformListener(b,n)
for _ in range(60): rclpy.spin_once(n,timeout_sec=0.1)
for f in ['panda_link0','base_footprint','robot0_agentview_left_optical_frame','robot0_eye_in_hand_optical_frame']:
    t=b.lookup_transform('world',f,rclpy.time.Time())
    tr,q=t.transform.translation,t.transform.rotation
    print(f, round(tr.x,4),round(tr.y,4),round(tr.z,4), '|', round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 10  ran 51.6s
cat > rb.py <<'EOF'
"""Shared helpers: frames, FK, snapshots, world<->link0."""
import numpy as np, rclpy, yaml
from pathlib import Path

M = yaml.safe_load(open('/workspace/machine.yaml'))
ARM = next(a for a in M['actuators'] if a['kind']=='joint_trajectory')['joints']

def quat_to_R(x,y,z,w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1.0)*2; w=0.25*s
        x=(R[2,1]-R[1,2])/s; y=(R[0,2]-R[2,0])/s; z=(R[1,0]-R[0,1])/s
    elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
        s=np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2; w=(R[2,1]-R[1,2])/s
        x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
    elif R[1,1]>R[2,2]:
        s=np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2; w=(R[0,2]-R[2,0])/s
        x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
    else:
        s=np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2; w=(R[1,0]-R[0,1])/s
        x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    q=np.array([x,y,z,w]); return q/np.linalg.norm(q)

class Rb:
    def __init__(self, name='rb'):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        from tf2_ros import Buffer, TransformListener
        self.buf = Buffer(); TransformListener(self.buf, self.node)
        self._clients = {}
        for _ in range(40): rclpy.spin_once(self.node, timeout_sec=0.05)

    def spin(self, n=10, t=0.05):
        for _ in range(n): rclpy.spin_once(self.node, timeout_sec=t)

    def tf(self, target, source):
        for _ in range(60):
            if self.buf.can_transform(target, source, rclpy.time.Time()): break
            rclpy.spin_once(self.node, timeout_sec=0.1)
        t = self.buf.lookup_transform(target, source, rclpy.time.Time())
        T = np.eye(4); q=t.transform.rotation; tr=t.transform.translation
        T[:3,:3] = quat_to_R(q.x,q.y,q.z,q.w); T[:3,3]=[tr.x,tr.y,tr.z]
        return T

    def T_w_l0(self):
        return self.tf('world','panda_link0')

    def w2l0(self, p):
        T = np.linalg.inv(self.T_w_l0()); return (T @ np.array([*p,1.0]))[:3]

    def l02w(self, p):
        T = self.T_w_l0(); return (T @ np.array([*p,1.0]))[:3]

    def grab(self, topic, mtype, timeout=20.0):
        got={}
        sub=self.node.create_subscription(mtype, topic, lambda m: got.setdefault('m',m), 1)
        import time
        end=time.time()+timeout
        while 'm' not in got and time.time()<end: rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return got.get('m')

    def joints(self):
        from sensor_msgs.msg import JointState
        m=self.grab('/joint_states', JointState)
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j=self.joints(); return [j[n] for n in ARM]

    def fingers(self):
        j=self.joints()
        return [v for k,v in j.items() if 'finger' in k]

    def fk(self, link='panda_hand', q=None):
        """Return 4x4 pose of link in WORLD frame via /compute_fk."""
        from moveit_msgs.srv import GetPositionFK
        from sensor_msgs.msg import JointState
        cli = self._clients.get('fk')
        if cli is None:
            cli = self.node.create_client(GetPositionFK, '/compute_fk')
            cli.wait_for_service(timeout_sec=15); self._clients['fk']=cli
        req = GetPositionFK.Request()
        req.header.frame_id = ''
        req.fk_link_names = [link]
        js = JointState(); js.name = list(ARM)
        js.position = list(q) if q is not None else list(self.arm_q())
        req.robot_state.joint_state = js
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f'FK failed {None if r is None else r.error_code.val}')
        ps = r.pose_stamped[0].pose
        T = np.eye(4)
        T[:3,:3] = quat_to_R(ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
        T[:3,3] = [ps.position.x, ps.position.y, ps.position.z]
        # result is in the planner model frame = panda_link0
        return self.T_w_l0() @ T

    def ik(self, T_world, seed=None, timeout=60.0, tries=1):
        """T_world: 4x4 desired panda_hand pose in WORLD. Returns joint list or None."""
        from moveit_msgs.srv import GetPositionIK
        from sensor_msgs.msg import JointState
        cli = self._clients.get('ik')
        if cli is None:
            cli = self.node.create_client(GetPositionIK, '/compute_ik')
            cli.wait_for_service(timeout_sec=15); self._clients['ik']=cli
        T_l0 = np.linalg.inv(self.T_w_l0()) @ T_world
        q = R_to_quat(T_l0[:3,:3])
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'panda_arm'
        req.ik_request.pose_stamped.header.frame_id = ''
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, T_l0[:3,3])
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        js = JointState(); js.name = list(ARM)
        js.position = list(seed) if seed is not None else list(self.arm_q())
        req.ik_request.robot_state.joint_state = js
        req.ik_request.timeout.sec = 2
        for _ in range(tries):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[n] for n in ARM]
        return None

    def move(self, positions, seconds=4.0, wait=True):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        cli = self._clients.get('fjt')
        if cli is None:
            cli = ActionClient(self.node, FollowJointTrajectory,
                               '/panda_arm_controller/follow_joint_trajectory')
            cli.wait_for_server(timeout_sec=15); self._clients['fjt']=cli
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = positions if isinstance(positions[0], (list,tuple,np.ndarray)) else [positions]
        n = len(pts)
        for i,pp in enumerate(pts):
            pt = JointTrajectoryPoint(positions=[float(v) for v in pp])
            tt = seconds*(i+1)/n
            pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt%1)*1e9))
            goal.trajectory.points.append(pt)
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        h = send.result()
        if not h.accepted: return 'rejected'
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        return rf.result().result.error_code

    def gripper(self, width):
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient
        cli = self._clients.get('grip')
        if cli is None:
            cli = ActionClient(self.node, GripperCommand, '/franka_gripper/gripper_action')
            cli.wait_for_server(timeout_sec=15); self._clients['grip']=cli
        g = GripperCommand.Goal(); g.command.position=float(width); g.command.max_effort=30.0
        f = cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, f)
        h=f.result(); rf=h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r=rf.result().result
        return (r.reached_goal, r.stalled, r.position)

    def snap(self, cam, out=None):
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        import cv2
        m = self.grab(f'/{cam}/color/image_raw', Image)
        img = CvBridge().imgmsg_to_cv2(m, 'bgr8')
        if out: cv2.imwrite(out, img)
        return img

    def depth(self, cam):
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        m = self.grab(f'/{cam}/depth/image_raw', Image)
        return CvBridge().imgmsg_to_cv2(m, 'passthrough')

    def info(self, cam):
        from sensor_msgs.msg import CameraInfo
        return self.grab(f'/{cam}/color/camera_info', CameraInfo)

    def px2w(self, cam, u, v, d=None):
        if d is None:
            dep = self.depth(cam); d = float(dep[int(v),int(u)])
        k = self.info(cam).k
        fx,fy,cx,cy = k[0],k[4],k[2],k[5]
        p = np.array([(u-cx)*d/fx, (v-cy)*d/fy, d, 1.0])
        T = self.tf('world', f'{cam}_optical_frame')
        return (T@p)[:3]
EOF
echo ok

# openrua op 11  ran 15.6s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r = Rb('survey')
for cam in ['robot0_agentview_left','robot0_agentview_right']:
    img = r.snap(cam); dep = r.depth(cam)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # bright blue mug
    mask = cv2.inRange(hsv, (95,120,120), (115,255,255))
    n,lab,stats,cent = cv2.connectedComponentsWithStats(mask,8)
    print('==',cam,'blue blobs:')
    for i in range(1,n):
        if stats[i,4] < 60: continue
        x,y,w,h,a = stats[i]
        print('  bbox',x,y,w,h,'area',a,'centroid',np.round(cent[i],1))
        # world of centroid
        u,v = int(cent[i][0]), int(cent[i][1])
        print('   center world', np.round(r.px2w(cam,u,v),4), 'depth',dep[v,u])
" 2>&1 | grep -v IP_LOCATOR

# openrua op 12  ran 17.6s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r = Rb('survey2')
pts_all=[]
for cam in ['robot0_agentview_left','robot0_agentview_right']:
    img = r.snap(cam); dep = r.depth(cam).astype(np.float32)
    k = r.info(cam).k; fx,fy,cx,cy = k[0],k[4],k[2],k[5]
    T = r.tf('world', cam+'_optical_frame')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (95,120,120), (115,255,255))
    vs,us = np.nonzero(mask)
    d = dep[vs,us]
    ok = np.isfinite(d)&(d>0)
    vs,us,d = vs[ok],us[ok],d[ok]
    P = np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)])
    W = (T@P)[:3].T
    pts_all.append(W)
    print(cam, 'n',len(W))
    print('  x',np.round([W[:,0].min(),W[:,0].max()],4),'y',np.round([W[:,1].min(),W[:,1].max()],4),'z',np.round([W[:,2].min(),W[:,2].max()],4))
W=np.vstack(pts_all)
print('MERGED mug: x',np.round([W[:,0].min(),W[:,0].max()],4),'y',np.round([W[:,1].min(),W[:,1].max()],4),'z',np.round([W[:,2].min(),W[:,2].max()],4))
print('center xy', np.round(W[:,0].mean(),4), np.round(W[:,1].mean(),4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 13  ran 45.1s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r = Rb('survey3')
cam='robot0_agentview_left'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
H,W=dep.shape
vs,us=np.mgrid[0:H,0:W]
d=dep.copy(); ok=np.isfinite(d)&(d>0)&(d<4)
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
np.save('wl.npy',Wc); np.save('okl.npy',ok)
def q(u,v,lbl=''):
    print(f'{lbl} px({u},{v}) -> ', np.round(Wc[:,v,u],4), 'd=',round(float(d[v,u]),3))
# counter surface samples
q(200,330,'counter-front-left'); q(300,320,'counter-mid'); q(250,300,'counter-back')
q(150,160,'backsplash-bottom-left'); q(200,155,'ledge?')
# shelf next to mug
q(300,120,'shelf-left-of-mug'); q(355,118,'shelf-right-of-mug'); q(280,105,'green-object')
q(323,124,'mug-bottom-px'); q(323,97,'mug-top-px')
q(320,60,'above-mug'); q(320,20,'high-above-mug')
print('counter z histogram of region rows 280-400 cols 100-350:')
reg=Wc[2,280:400,100:350][ok[280:400,100:350]]
print(np.round(np.percentile(reg,[5,25,50,75,95]),4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 14  ran 74.8s
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb, R_to_quat
r=Rb('fkchk')
Th=r.fk('panda_hand')
Tl8=r.fk('panda_link8')
print('hand world:\n',np.round(Th,4))
Tc=r.tf('world','robot0_eye_in_hand_optical_frame')
print('cam world:\n',np.round(Tc,4))
H2C=np.linalg.inv(Th)@Tc
print('hand->cam:\n',np.round(H2C,4))
print('q hand', np.round(R_to_quat(Th[:3,:3]),4))
print('arm q', np.round(r.arm_q(),4))
print('fingers', np.round(r.fingers(),4))
print('T_w_l0:\n', np.round(r.T_w_l0(),4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 15  ran 25.2s
timeout 300 python3 -u -c "
import numpy as np
W=np.load('wl.npy'); ok=np.load('okl.npy')
P=W[:,ok].T
print('cloud n',len(P))
# counter surface
c=P[(np.abs(P[:,2]-0.922)<0.01)]
print('counter pts',len(c),'y range',np.round([c[:,1].min(),c[:,1].max()],4),'x range',np.round([c[:,0].min(),c[:,0].max()],4))
# shelf surface 1.422
s=P[(np.abs(P[:,2]-1.422)<0.01)]
print('shelf pts',len(s),'y range',np.round([s[:,1].min(),s[:,1].max()],4),'x range',np.round([s[:,0].min(),s[:,0].max()],4))
# what is below counter front edge: pts with z<0.92
lo=P[P[:,2]<0.90]
print('below-counter pts',len(lo))
if len(lo): print(' y range',np.round([lo[:,1].min(),lo[:,1].max()],4),'z range',np.round([lo[:,2].min(),lo[:,2].max()],4))
# z histogram overall
print('z percentiles', np.round(np.percentile(P[:,2],[1,5,10,25,50,75,90,99]),3))
# cabinet/shelf structure: points with z between 1.45 and 1.9
hi=P[(P[:,2]>1.5)]
print('z>1.5 pts',len(hi))
if len(hi): print(' y range',np.round([hi[:,1].min(),hi[:,1].max()],4),' z max',round(hi[:,2].max(),3))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 16  ran 22.3s
timeout 300 python3 -u -c "
import numpy as np
W=np.load('wl.npy'); ok=np.load('okl.npy')
# per-x slice around mug column
for xlo,xhi,lbl in [(0.9,1.4,'x0.9-1.4'),(0.4,0.9,'x0.4-0.9'),(1.4,1.9,'x1.4-1.9')]:
    m = ok & (W[0]>xlo)&(W[0]<xhi)&(np.abs(W[2]-0.922)<0.012)
    P=W[:,m].T
    if len(P): print(lbl,'counter y range',np.round([P[:,1].min(),P[:,1].max()],4),'n',len(P))
print()
# top-down occupancy: for x in 0.9-1.4, print min y of any point with z in 0.5..0.9 (cabinet front)
for zlo,zhi in [(0.1,0.5),(0.5,0.9),(0.93,1.2),(1.2,1.42)]:
    m = ok & (W[0]>0.9)&(W[0]<1.4)&(W[2]>zlo)&(W[2]<zhi)
    P=W[:,m].T
    if len(P): print(f'z {zlo}-{zhi}: y min {P[:,1].min():.3f} max {P[:,1].max():.3f} n {len(P)}')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 17  ran 18.6s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('cm')
for cam in ['robot0_agentview_left','robot0_agentview_right']:
    img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
    k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    T=r.tf('world',cam+'_optical_frame')
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(5,60,60),(25,200,230))
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask,8)
    print('==',cam)
    for i in range(1,n):
        if stats[i,4]<400: continue
        x,y,w,h,a=stats[i]
        vs,us=np.nonzero(lab==i); d=dep[vs,us]
        okm=np.isfinite(d)&(d>0); vs,us,d=vs[okm],us[okm],d[okm]
        P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)])
        Wp=(T@P)[:3].T
        print(' blob bbox',x,y,w,h,'area',a)
        print('   world x',np.round([Wp[:,0].min(),Wp[:,0].max()],3),'y',np.round([Wp[:,1].min(),Wp[:,1].max()],3),'z',np.round([Wp[:,2].min(),Wp[:,2].max()],3))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 18  ran 28.6s
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb
W=np.load('wl.npy'); ok=np.load('okl.npy')
def q(u,v,lbl):
    print(f'{lbl:26s} px({u},{v}) ->', np.round(W[:,v,u],4), 'ok' if ok[v,u] else 'BAD')
q(370,276,'drip-tray-pad-center')
q(370,285,'pad-lower')
q(365,265,'pad-upper')
q(380,240,'spout-area')
q(392,214,'button-upper')
q(392,231,'button-lower')
q(378,175,'machine-top')
q(417,265,'black-ring-center')
q(400,250,'ring-left')
q(437,265,'ring-right')
q(350,300,'machine-base-front')
q(355,175,'machine-body-topleft')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 19  ran 47.9s
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('poses')
cands={
 'home': [0,-0.785,0,-2.356,0,1.571,0.785],
 'stow_back': [0,-1.2,0,-2.6,0,1.6,0.785],
 'stow_left': [-1.5,-1.0,0,-2.4,0,1.6,0.785],
 'stow_right':[1.5,-1.0,0,-2.4,0,1.6,0.785],
 'up': [0,-0.3,0,-1.5,0,1.2,0.785],
}
for k,q in cands.items():
    T=r.fk('panda_hand',q)
    print(f'{k:11s} hand world {np.round(T[:3,3],3)} approach {np.round(T[:3,2],2)}')
    T7=r.fk('panda_link4',q); T5=r.fk('panda_link6',q)
    print(f'            link4 {np.round(T7[:3,3],3)}  link6 {np.round(T5[:3,3],3)}')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 20  ran 9.1s
python3 - <<'EOF'
import re
s = open('rb.py').read()
s = s.replace("js.position = list(q) if q is not None else list(self.arm_q())",
              "js.position = [float(v) for v in (q if q is not None else self.arm_q())]")
s = s.replace("js.position = list(seed) if seed is not None else list(self.arm_q())",
              "js.position = [float(v) for v in (seed if seed is not None else self.arm_q())]")
open('rb.py','w').write(s)
EOF
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('poses')
cands={
 'home': [0,-0.785,0,-2.356,0,1.571,0.785],
 'stow_back': [0,-1.2,0,-2.6,0,1.6,0.785],
 'stow_left': [-1.5,-1.0,0,-2.4,0,1.6,0.785],
 'stow_right':[1.5,-1.0,0,-2.4,0,1.6,0.785],
 'up': [0,-0.3,0,-1.5,0,1.2,0.785],
}
for k,q in cands.items():
    T=r.fk('panda_hand',q)
    T4=r.fk('panda_link4',q); T6=r.fk('panda_link6',q)
    print(f'{k:11s} hand {np.round(T[:3,3],3)} appr {np.round(T[:3,2],2)} link4 {np.round(T4[:3,3],3)} link6 {np.round(T6[:3,3],3)}')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 21  ran 76.8s
cat > plan.py <<'EOF'
#!/usr/bin/env python3
"""plan.py: IK to a world hand pose, report, optionally execute.

Usage: plan.py x y z [--yaw deg] [--pitch deg] [--exec] [--secs S]
Orientation: top-down by default (hand +Z = world -Z, i.e. approach down).
  --yaw   rotates the finger axis in the horizontal plane (0 = fingers
          close along world +X).
  --pitch tilts the approach away from straight-down toward -Y (0 = down,
          90 = horizontal pointing +Y).
"""
import sys, numpy as np
from rb import Rb, R_to_quat

def hand_R(yaw_deg=0.0, pitch_deg=0.0):
    y = np.deg2rad(yaw_deg); p = np.deg2rad(pitch_deg)
    # start: approach straight down
    hz = np.array([0,0,-1.0]); hy = np.array([1.0,0,0]); hx = np.cross(hy,hz)
    R = np.stack([hx,hy,hz],axis=1)
    # yaw about world Z
    c,s = np.cos(y), np.sin(y)
    Rz = np.array([[c,-s,0],[s,c,0],[0,0,1.0]])
    R = Rz @ R
    # pitch: rotate about the current finger axis (hand Y) so approach
    # tilts from -Z toward +Y
    ax = R[:,1]/np.linalg.norm(R[:,1])
    K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    Rp = np.eye(3) + np.sin(p)*K + (1-np.cos(p))*(K@K)
    return Rp @ R

def main():
    a = sys.argv[1:]
    x,y,z = float(a[0]),float(a[1]),float(a[2])
    def opt(n,d):
        return float(a[a.index(n)+1]) if n in a else d
    yaw = opt('--yaw',0.0); pitch = opt('--pitch',0.0); secs = opt('--secs',4.0)
    do = '--exec' in a
    r = Rb('plan')
    T = np.eye(4); T[:3,:3] = hand_R(yaw,pitch); T[:3,3] = [x,y,z]
    print('target hand R:\n', np.round(T[:3,:3],3))
    l0 = np.linalg.inv(r.T_w_l0()) @ T
    sh = l0[:3,3] - np.array([0,0,0.333])
    print(f'link0 pos {np.round(l0[:3,3],4)}  |shoulder-dist| {np.linalg.norm(sh):.4f}')
    q = r.ik(T)
    if q is None:
        print('IK FAILED'); sys.exit(2)
    print('q =', np.round(q,4))
    Tf = r.fk('panda_hand', q)
    print('FK hand pos', np.round(Tf[:3,3],4), 'err', round(float(np.linalg.norm(Tf[:3,3]-T[:3,3])),4))
    print('FK approach', np.round(Tf[:3,2],3))
    for lk in ['panda_link3','panda_link4','panda_link5','panda_link6','panda_link7']:
        print(f'  {lk} {np.round(r.fk(lk,q)[:3,3],3)}')
    if do:
        code = r.move(q, secs)
        print('move error_code =', code)
        cur = r.arm_q()
        print('reached q ', np.round(cur,4))
        print('max joint err', round(float(np.max(np.abs(np.array(cur)-np.array(q)))),4))
        Tn = r.fk('panda_hand')
        print('actual hand pos', np.round(Tn[:3,3],4), 'approach', np.round(Tn[:3,2],3))

main()
EOF
timeout 300 python3 -u plan.py 1.13 -0.295 1.60 2>&1 | grep -v IP_LOCATOR

# openrua op 22  ran 439.8s
timeout 600 python3 -u -c "
import numpy as np
from rb import Rb
import plan as P
" 2>&1 | head -3
cat > sweep.py <<'EOF'
import sys, numpy as np
from rb import Rb
def hand_R(yaw_deg=0.0, pitch_deg=0.0):
    y=np.deg2rad(yaw_deg); p=np.deg2rad(pitch_deg)
    hz=np.array([0,0,-1.0]); hy=np.array([1.0,0,0]); hx=np.cross(hy,hz)
    R=np.stack([hx,hy,hz],axis=1)
    c,s=np.cos(y),np.sin(y); Rz=np.array([[c,-s,0],[s,c,0],[0,0,1.0]]); R=Rz@R
    ax=R[:,1]/np.linalg.norm(R[:,1])
    K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    Rp=np.eye(3)+np.sin(p)*K+(1-np.cos(p))*(K@K)
    return Rp@R
r=Rb('sweep')
seed=r.arm_q()
best=[]
for z in [1.60,1.58,1.62,1.56]:
  for yv in [-0.295,-0.33,-0.26]:
    for yaw in [0,45,90,-45,135,180]:
      for pitch in [0,15,30]:
        T=np.eye(4); T[:3,:3]=hand_R(yaw,pitch); T[:3,3]=[1.13,yv,z]
        q=r.ik(T,seed=seed)
        if q is not None:
            Tf=r.fk('panda_hand',q); e=np.linalg.norm(Tf[:3,3]-T[:3,3])
            print(f'OK z={z} y={yv} yaw={yaw} pitch={pitch} err={e:.4f} q={np.round(q,3)}')
            best.append((z,yv,yaw,pitch))
print('total OK', len(best))
EOF
timeout 900 python3 -u sweep.py 2>&1 | grep -v IP_LOCATOR

# openrua op 23  ran 96.1s
timeout 900 python3 -u plan.py 1.13 -0.33 1.58 --yaw 0 --pitch 30 --exec --secs 4 2>&1 | grep -v IP_LOCATOR

# openrua op 24  ran 7.3s
timeout 300 python3 -u -c "
from rb import Rb
r=Rb('look')
r.snap('robot0_eye_in_hand','eih1.png')
r.snap('robot0_agentview_left','avl1.png')
print('ok')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 25  ran 49.2s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('mugmeas')
cam='robot0_eye_in_hand'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
print('cam T:\n',np.round(T,4))
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
H,W=dep.shape
vs,us=np.mgrid[0:H,0:W]
d=dep
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d,np.ones_like(d)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
np.save('eih_w.npy',Wc); cv2.imwrite('eih_dbg.png',img)
blue=cv2.inRange(hsv,(95,120,120),(115,255,255))
dark=cv2.inRange(hsv,(115,80,20),(145,255,120))  # purple interior
print('blue px',blue.sum()//255,'dark px',dark.sum()//255)
for name,m in [('blue',blue),('dark',dark)]:
    n,lab,stats,cent=cv2.connectedComponentsWithStats(m,8)
    for i in range(1,n):
        if stats[i,4]<300: continue
        vv,uu=np.nonzero(lab==i); dd=d[vv,uu]
        good=np.isfinite(dd)&(dd>0)
        pts=Wc[:,vv[good],uu[good]].T
        print(f'{name} blob bbox {stats[i,:4]} area {stats[i,4]}')
        print('   x',np.round([pts[:,0].min(),pts[:,0].max()],4),'y',np.round([pts[:,1].min(),pts[:,1].max()],4),'z',np.round([pts[:,2].min(),pts[:,2].max()],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 26  ran 26.8s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('mug2')
cam='robot0_eye_in_hand'
img=cv2.imread('eih_dbg.png'); Wc=np.load('eih_w.npy')
dep=r.depth(cam).astype(np.float32)
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
blue=cv2.inRange(hsv,(95,120,120),(115,255,255))>0
dark=cv2.inRange(hsv,(115,80,20),(145,255,120))>0
ok=np.isfinite(dep)&(dep>0)
def fit_circle(P):
    x,y=P[:,0],P[:,1]
    A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    cx,cy=c[0]/2,c[1]/2; R=np.sqrt(c[2]+cx**2+cy**2)
    return cx,cy,R
for lbl,m in [('blue',blue),('dark',dark),('union',blue|dark)]:
    for zt in [1.4740,1.4700,1.4650]:
        sel=m&ok&(Wc[2]>zt)
        P=Wc[:,sel].T
        if len(P)<20: continue
        cx,cy,R=fit_circle(P)
        print(f'{lbl:6s} z>{zt}: n={len(P):5d} xrange {P[:,0].min():.4f},{P[:,0].max():.4f} yrange {P[:,1].min():.4f},{P[:,1].max():.4f} circle c=({cx:.4f},{cy:.4f}) R={R:.4f}')
    print()
# z profile of the body (blue) - horizontal extents per z band
print('blue body cross sections:')
for z0 in np.arange(1.420,1.480,0.008):
    sel=blue&ok&(Wc[2]>=z0)&(Wc[2]<z0+0.008)
    P=Wc[:,sel].T
    if len(P)<20: continue
    print(f'  z {z0:.3f}-{z0+0.008:.3f} n={len(P):5d} x {P[:,0].min():.4f}..{P[:,0].max():.4f} ({P[:,0].max()-P[:,0].min():.4f}) y {P[:,1].min():.4f}..{P[:,1].max():.4f} ({P[:,1].max()-P[:,1].min():.4f})')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 27  ran 122.9s
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('fingers')
q=r.arm_q()
Th=r.fk('panda_hand',q)
for lk in ['panda_leftfinger','panda_rightfinger','panda_hand_tcp','panda_link8']:
    try:
        T=r.fk(lk,q)
        d=np.linalg.inv(Th)@T
        print(f'{lk:20s} world {np.round(T[:3,3],4)}  in-hand {np.round(d[:3,3],4)}')
    except Exception as e:
        print(lk,'ERR',str(e)[:80])
" 2>&1 | grep -v IP_LOCATOR

# openrua op 28  ran 8.7s
timeout 180 python3 -u -c "
import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf')
q=QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
got=[]
n.create_subscription(String,'/robot_description',lambda m:got.append(m.data),q)
import time
end=time.time()+20
while not got and time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
open('/workspace/robot.urdf','w').write(got[0] if got else '')
print('len',len(got[0]) if got else 0)
" 2>&1 | grep -v IP_LOCATOR
grep -n -A25 'name="panda_leftfinger"' /workspace/robot.urdf | head -40

# openrua op 29  ran 83.1s
cat > pose.py <<'EOF'
import numpy as np
TCP = 0.1034
def rim_pose(phi_deg, tilt_deg):
    """finger axis = radial dir at angle phi; approach = -Z rotated about
    that axis by tilt. Returns 3x3 R with columns [handX,handY,handZ]."""
    p = np.deg2rad(phi_deg); t = np.deg2rad(tilt_deg)
    f = np.array([np.cos(p), np.sin(p), 0.0])
    a = np.array([-np.sin(t)*np.sin(p), np.sin(t)*np.cos(p), -np.cos(t)])
    hx = np.cross(f, a)
    return np.stack([hx, f, a], axis=1), a
def hand_T(tcp_xyz, phi_deg, tilt_deg, tcp=TCP):
    R, a = rim_pose(phi_deg, tilt_deg)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = np.array(tcp_xyz) - tcp*a
    return T, a
EOF
cat > gsweep.py <<'EOF'
import numpy as np, itertools
from rb import Rb
from pose import hand_T
r = Rb('gsweep')
C = np.array([1.1308, -0.2396]); ZTOP = 1.4772
seed = r.arm_q()
res=[]
for phi in [0, -20, 20, -40, 40, -60]:
    for tilt in [0, 15, 25, 35, 45]:
        rad = 0.0395
        P = np.array([C[0]+rad*np.cos(np.deg2rad(phi)), C[1]+rad*np.sin(np.deg2rad(phi)), ZTOP-0.022])
        Tg,a = hand_T(P, phi, tilt)
        Tp,_ = hand_T(P - 0.08*a, phi, tilt)
        qp = r.ik(Tp, seed=seed)
        if qp is None: continue
        qg = r.ik(Tg, seed=qp)
        if qg is None:
            print(f'phi={phi:4d} tilt={tilt:3d}  PRE ok, GRASP fail'); continue
        d = np.max(np.abs(np.array(qg)-np.array(qp)))
        eg = np.linalg.norm(r.fk('panda_hand',qg)[:3,3]-Tg[:3,3])
        print(f'phi={phi:4d} tilt={tilt:3d}  BOTH OK  dq={d:.3f} err={eg:.4f}')
        print(f'   qpre  {np.round(qp,4)}')
        print(f'   qgra  {np.round(qg,4)}')
        res.append((phi,tilt,qp,qg,d))
print('feasible:',len(res))
np.save('gsweep.npy', np.array([(p,t,dd) for p,t,_,_,dd in res]))
EOF
timeout 1500 python3 -u gsweep.py 2>&1 | grep -v IP_LOCATOR

# openrua op 30  ran 73.6s
cat > look.py <<'EOF'
#!/usr/bin/env python3
"""look.py cx cy cz tx ty tz [--exec] [--secs S] [--tag NAME]
Place the wrist camera at (cx,cy,cz) aimed at (tx,ty,tz)."""
import sys, numpy as np
from rb import Rb
CAMOFF = 0.05  # camera sits +0.05 along hand X

def hand_T_for_view(cam, tgt):
    cam=np.array(cam,float); tgt=np.array(tgt,float)
    cz = tgt-cam; cz/=np.linalg.norm(cz)
    cx = np.cross(cz, np.array([0,0,1.0]))
    if np.linalg.norm(cx) < 1e-6: cx = np.array([1.0,0,0])
    cx/=np.linalg.norm(cx)
    cy = np.cross(cz,cx)
    hx, hy, hz = -cy, cx, cz
    T=np.eye(4); T[:3,:3]=np.stack([hx,hy,hz],1); T[:3,3]=cam-CAMOFF*hx
    return T

if __name__ == '__main__':
    a=sys.argv[1:]
    cam=[float(v) for v in a[0:3]]; tgt=[float(v) for v in a[3:6]]
    secs=float(a[a.index('--secs')+1]) if '--secs' in a else 4.0
    tag=a[a.index('--tag')+1] if '--tag' in a else 'look'
    r=Rb('look')
    T=hand_T_for_view(cam,tgt)
    l0=np.linalg.inv(r.T_w_l0())@T
    print('hand pos',np.round(T[:3,3],4),'shoulder dist',round(float(np.linalg.norm(l0[:3,3]-[0,0,0.333])),4))
    q=r.ik(T)
    if q is None: print('IK FAILED'); sys.exit(2)
    print('q',np.round(q,4))
    if '--exec' in a:
        print('move code', r.move(q,secs))
        Tn=r.fk('panda_hand'); print('actual hand',np.round(Tn[:3,3],4))
        r.snap('robot0_eye_in_hand',f'{tag}.png')
        print('saved',tag+'.png')
EOF
timeout 900 python3 -u look.py 1.28 -0.75 1.40 1.27 -0.38 1.05 2>&1 | grep -v IP_LOCATOR

# openrua op 31  ran 13.5s
timeout 900 python3 -u -c "
import numpy as np
from rb import Rb
from look import hand_T_for_view
r=Rb('lk')
cands=[((1.28,-0.75,1.40),(1.27,-0.38,1.05)),
       ((1.28,-0.85,1.55),(1.27,-0.35,1.05)),
       ((1.28,-0.80,1.50),(1.27,-0.36,1.08)),
       ((1.10,-0.80,1.45),(1.27,-0.36,1.08)),
       ((1.45,-0.80,1.45),(1.27,-0.36,1.08))]
for cam,tgt in cands:
    T=hand_T_for_view(cam,tgt)
    q=r.ik(T)
    if q is None: print(cam,'IKFAIL'); continue
    ls={lk:np.round(r.fk(lk,q)[:3,3],3) for lk in ['panda_link3','panda_link5','panda_link7','panda_hand']}
    print(cam,'->',np.round(q,3))
    print('   ',ls)
" 2>&1 | grep -v IP_LOCATOR

# openrua op 32  ran 90.0s
timeout 900 python3 -u look.py 1.28 -0.80 1.50 1.27 -0.36 1.08 --exec --secs 5 --tag cm_view 2>&1 | grep -v IP_LOCATOR

# openrua op 33  ran 25.6s
timeout 300 python3 -u -c "
import numpy as np
from rb import Rb
from look import hand_T_for_view
r=Rb('chk')
Th=r.fk('panda_hand')
Tc=r.tf('world','robot0_eye_in_hand_optical_frame')
print('hand:\n',np.round(Th,4))
print('cam:\n',np.round(Tc,4))
print('hand->cam:\n',np.round(np.linalg.inv(Th)@Tc,4))
Tw=hand_T_for_view((1.28,-0.80,1.50),(1.27,-0.36,1.08))
print('intended hand R:\n',np.round(Tw[:3,:3],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 34  ran 87.3s
python3 - <<'EOF'
s=open('rb.py').read()
old="        T_l0 = np.linalg.inv(self.T_w_l0()) @ T_world"
new="""        # machine fact: /compute_ik solves for panda_link8, not panda_hand.
        # panda_hand = link8 * Rz(-45deg), so ask for hand_pose * Rz(+45deg).
        c = np.sqrt(0.5)
        RZ45 = np.array([[c,-c,0,0],[c,c,0,0],[0,0,1,0],[0,0,0,1.0]])
        T_l0 = np.linalg.inv(self.T_w_l0()) @ (np.asarray(T_world) @ RZ45)"""
assert old in s
s=s.replace(old,new)
open('rb.py','w').write(s)
EOF
timeout 900 python3 -u -c "
import numpy as np
from rb import Rb
from look import hand_T_for_view
r=Rb('chk2')
T=hand_T_for_view((1.28,-0.80,1.50),(1.27,-0.36,1.08))
q=r.ik(T)
print('q',np.round(q,4))
Tf=r.fk('panda_hand',q)
print('FK hand R:\n',np.round(Tf[:3,:3],4))
print('want   R:\n',np.round(T[:3,:3],4))
print('pos err',round(float(np.linalg.norm(Tf[:3,3]-T[:3,3])),5),'rot err',round(float(np.linalg.norm(Tf[:3,:3]-T[:3,:3])),5))
print('move',r.move(q,4.0))
r.snap('robot0_eye_in_hand','cm_view2.png')
print('saved')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 35  ran 7.1s
python3 -c "
import cv2
img=cv2.imread('cm_view2.png')
cv2.imwrite('cm_zoom.png', cv2.resize(img[100:330, 265:390], None, fx=5, fy=5, interpolation=cv2.INTER_CUBIC))
"

# openrua op 36  ran 24.5s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('cmmeas')
cam='robot0_eye_in_hand'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
np.save('cm_w.npy',Wc); cv2.imwrite('cm_rgb.png',img)
def q(u,v,l): print(f'{l:22s} px({u},{v}) ->',np.round(Wc[:,v,u],4),' d=',round(float(dep[v,u]),4))
q(322,210,'button-upper'); q(316,210,'button-upper-L'); q(328,210,'button-upper-R')
q(322,237,'button-lower')
q(322,247,'spout-dark'); q(322,255,'spout-lower')
q(322,285,'drip-tray-center'); q(300,285,'tray-L'); q(345,285,'tray-R')
q(322,270,'tray-back'); q(322,300,'tray-front')
q(322,190,'body-front-mid'); q(322,120,'body-top')
q(285,285,'tray-far-L'); q(358,285,'tray-far-R')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 37  ran 100.5s
timeout 300 python3 -u -c "
import numpy as np, cv2
Wc=np.load('cm_w.npy'); img=cv2.imread('cm_rgb.png')
fin=np.isfinite(Wc).all(0)
# tray plane z~0.974
m=fin&(np.abs(Wc[2]-0.9741)<0.004)&(Wc[0]>1.1)&(Wc[0]<1.45)&(Wc[1]>-0.55)&(Wc[1]<-0.2)
P=Wc[:,m].T
print('tray pts',len(P),'x',np.round([P[:,0].min(),P[:,0].max()],4),'y',np.round([P[:,1].min(),P[:,1].max()],4))
# per-x the y extent
for x0 in np.arange(1.20,1.36,0.02):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.02)]
    if len(s)>5: print(f'  x {x0:.2f}-{x0+0.02:.2f}: y {s[:,1].min():.4f}..{s[:,1].max():.4f} n={len(s)}')
print()
# anything above tray between z 0.98 and 1.10 near the machine (side walls / plinth)
m2=fin&(Wc[2]>0.978)&(Wc[2]<1.10)&(Wc[0]>1.15)&(Wc[0]<1.40)&(Wc[1]>-0.50)&(Wc[1]<-0.15)
P2=Wc[:,m2].T
print('obstacles above tray z0.98-1.10:',len(P2))
if len(P2):
  for z0 in np.arange(0.98,1.10,0.02):
    s=P2[(P2[:,2]>=z0)&(P2[:,2]<z0+0.02)]
    if len(s)>5: print(f'  z {z0:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)}')
print()
m3=fin&(Wc[2]>1.10)&(Wc[2]<1.16)&(Wc[0]>1.15)&(Wc[0]<1.40)&(Wc[1]>-0.50)&(Wc[1]<-0.15)
P3=Wc[:,m3].T
print('dispenser head z1.10-1.16:',len(P3))
if len(P3): print('  x',np.round([P3[:,0].min(),P3[:,0].max()],3),'y',np.round([P3[:,1].min(),P3[:,1].max()],3))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 38  ran 132.1s
python3 -c "
import cv2
img=cv2.imread('cm_rgb.png')
cv2.imwrite('btn_zoom.png', cv2.resize(img[195:250, 295:350], None, fx=12, fy=12, interpolation=cv2.INTER_CUBIC))
print(img.shape)
"

# openrua op 39  ran 67.2s
timeout 1500 python3 -u gsweep.py 2>&1 | grep -v IP_LOCATOR

# openrua op 40  ran 43.0s
cat > scene.py <<'EOF'
"""Measured obstacle boxes (world frame) + swept-path checker."""
import numpy as np
from rb import Rb

BOXES = {
 # name: (xmin,xmax, ymin,ymax, zmin,zmax)
 'counter':   (-0.5, 2.6, -0.905, 0.05, -0.1, 0.922),
 'backwall':  (-0.5, 2.6, -0.02, 0.6, 0.90, 2.2),
 'cabinet':   (-0.5, 2.6, -0.255, 0.05, 1.20, 1.425),   # shelf slab + front
 'coffee':    (1.185, 1.352, -0.465, -0.125, 0.90, 1.270),
 'knives':    (1.040, 1.180, -0.350, -0.040, 0.90, 1.165),
 'toaster':   (1.480, 1.760, -0.480, -0.120, 0.90, 1.130),
}
LINKS = ['panda_link1','panda_link2','panda_link3','panda_link4',
         'panda_link5','panda_link6','panda_link7','panda_hand']

def pts_for(r, q):
    """Sample points along the arm: link origins + segment midpoints +
    the two fingertips + the hand's far corners."""
    Ts = {l: r.fk(l, q) for l in LINKS}
    P = [Ts[l][:3,3] for l in LINKS]
    for a,b in zip(LINKS[:-1],LINKS[1:]):
        pa,pb = Ts[a][:3,3], Ts[b][:3,3]
        for t in (0.33,0.66): P.append(pa+(pb-pa)*t)
    Th = Ts['panda_hand']; o=Th[:3,3]; hx,hy,hz = Th[:3,0],Th[:3,1],Th[:3,2]
    for sy in (-0.045,0.0,0.045):
        for d in (0.03,0.0584,0.08,0.1034):
            P.append(o + hy*sy + hz*d)
    return np.array(P), Ts

def hits(P, margin=0.015, skip=()):
    out=[]
    for name,(x0,x1,y0,y1,z0,z1) in BOXES.items():
        if name in skip: continue
        m = ((P[:,0]>x0-margin)&(P[:,0]<x1+margin)&
             (P[:,1]>y0-margin)&(P[:,1]<y1+margin)&
             (P[:,2]>z0-margin)&(P[:,2]<z1+margin))
        if m.any(): out.append((name, int(m.sum()), np.round(P[m][0],3)))
    return out

def check_path(r, qa, qb, n=9, margin=0.015, skip=(), label=''):
    qa=np.array(qa,float); qb=np.array(qb,float); bad=[]
    for i in range(n+1):
        q = qa + (qb-qa)*i/n
        P,_ = pts_for(r,q)
        h = hits(P, margin, skip)
        if h: bad.append((round(i/n,2), h))
    if bad:
        print(f'  !! {label} COLLISIONS:')
        for t,h in bad: print(f'     t={t}: {h}')
    else:
        print(f'  ok {label}: clear (margin {margin})')
    return not bad
EOF
timeout 900 python3 -u -c "
import numpy as np
from rb import Rb
import scene
r=Rb('chk')
qcur=r.arm_q()
qpre=[-1.4264,-0.461,1.0323,-1.2157,0.6104,1.7908,0.6106]
qgra=[-1.5335,-0.4585,1.2112,-1.3139,0.6066,1.9557,0.5984]
for nm,q in [('current',qcur),('pre',qpre),('grasp',qgra)]:
    P,Ts=scene.pts_for(r,q)
    print(nm,'hand',np.round(Ts['panda_hand'][:3,3],3),'hits',scene.hits(P))
    print('   links',{k[-6:]:np.round(v[:3,3],3).tolist() for k,v in Ts.items()})
print()
scene.check_path(r,qcur,qpre,n=8,label='cur->pre')
scene.check_path(r,qpre,qgra,n=5,label='pre->grasp',skip=('cabinet',))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 41  ran 86.4s
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
import scene
r=Rb('grasp1')
print('gripper open ->', r.gripper(0.04))
print('fingers', np.round(r.fingers(),4))
qpre=[-1.4264,-0.461,1.0323,-1.2157,0.6104,1.7908,0.6106]
print('move code', r.move(qpre, 5.0))
q=r.arm_q(); print('q err', np.round(np.array(q)-np.array(qpre),4))
T=r.fk('panda_hand'); print('hand', np.round(T[:3,3],4)); print('R', np.round(T[:3,:3],3))
r.snap('robot0_eye_in_hand','pre_grasp.png')
print('saved')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 42  ran 29.9s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('verify')
cam='robot0_eye_in_hand'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
blue=cv2.inRange(hsv,(95,120,120),(115,255,255))>0
dark=cv2.inRange(hsv,(115,80,20),(145,255,120))>0
ok=np.isfinite(dep)&(dep>0)
def fit(P):
    x,y=P[:,0],P[:,1]; A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cx_,cy_=c[0]/2,c[1]/2
    return cx_,cy_,np.sqrt(c[2]+cx_**2+cy_**2)
zt=Wc[2][(blue|dark)&ok].max(); print('mug top z',round(float(zt),4))
for lbl,m in [('blue',blue),('dark',dark)]:
    sel=m&ok&(Wc[2]>zt-0.004)
    Pp=Wc[:,sel].T
    if len(Pp)>20:
        a,b,R=fit(Pp); print(f'{lbl} rim: center=({a:.4f},{b:.4f}) R={R:.4f} n={len(Pp)}')
# project predicted grasp geometry into image
Tc=np.linalg.inv(T)
def proj(p):
    q=Tc@np.array([*p,1.0]); 
    return int(q[0]/q[2]*fx+cx), int(q[1]/q[2]*fy+cy)
C=np.array([1.1308,-0.2396,1.4772])
vis=img.copy()
for ang in range(0,360,6):
    a=np.deg2rad(ang)
    for R,col in [(0.0403,(0,255,0)),(0.0376,(0,255,255))]:
        u,v=proj(C+np.array([R*np.cos(a),R*np.sin(a),0]))
        if 0<=u<640 and 0<=v<480: cv2.circle(vis,(u,v),1,col,-1)
Th=r.fk('panda_hand'); o=Th[:3,3]; hy=Th[:3,1]; hz=Th[:3,2]
for sy,col in [(-0.04,(0,0,255)),(0.04,(255,0,0))]:
    for d in np.arange(0.058,0.105,0.005):
        u,v=proj(o+hy*sy+hz*d)
        if 0<=u<640 and 0<=v<480: cv2.circle(vis,(u,v),2,col,-1)
u,v=proj(o+hz*0.1034); cv2.drawMarker(vis,(u,v),(255,255,255),cv2.MARKER_CROSS,14,2)
cv2.imwrite('overlay.png',vis); print('TCP proj',u,v)
" 2>&1 | grep -v IP_LOCATOR

# openrua op 43  ran 30.4s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('ov2')
cam='robot0_eye_in_hand'
img=r.snap(cam)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
Tc=np.linalg.inv(r.tf('world',cam+'_optical_frame'))
def proj(p):
    q=Tc@np.array([*p,1.0]); return int(round(q[0]/q[2]*fx+cx)), int(round(q[1]/q[2]*fy+cy))
C=np.array([1.1308,-0.2396,1.4772])
vis=img.copy()
for ang in range(0,360,5):
    a=np.deg2rad(ang)
    u,v=proj(C+np.array([0.0403*np.cos(a),0.0403*np.sin(a),0]))
    if 0<=u<640 and 0<=v<480: cv2.circle(vis,(u,v),1,(0,255,0),-1)
# grasp-time finger lines: from fingertip up along -hz for 5cm
hy=np.array([1.0,0,0]); hz=np.array([0.0,0.7071,-0.7071])
tcp=np.array([1.1703,-0.2396,1.4552])
for sy,col in [(-0.04,(0,0,255)),(0.04,(255,0,0))]:
    for d in np.arange(0,0.05,0.004):
        u,v=proj(tcp+hy*sy-hz*d)
        if 0<=u<640 and 0<=v<480: cv2.circle(vis,(u,v),2,col,-1)
u,v=proj(tcp); cv2.drawMarker(vis,(u,v),(255,255,255),cv2.MARKER_CROSS,16,2)
print('grasp TCP proj',u,v)
cv2.imwrite('overlay2.png',vis)
" 2>&1 | grep -v IP_LOCATOR

# openrua op 44  ran 55.5s
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('grasp2')
qgra=[-1.5335,-0.4585,1.2112,-1.3139,0.6066,1.9557,0.5984]
print('move', r.move(qgra,2.5))
T=r.fk('panda_hand'); print('hand',np.round(T[:3,3],4))
print('TCP',np.round(T[:3,3]+T[:3,2]*0.1034,4))
print('close ->', r.gripper(0.0))
f=r.fingers(); print('fingers',np.round(f,5),'gap',round(float(abs(f[0])+abs(f[1])),5))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 45  ran 26.6s
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('lift')
T=r.fk('panda_hand'); Tn=T.copy(); Tn[2,3]+=0.09
q=r.ik(Tn)
print('ik',None if q is None else np.round(q,4))
print('move', r.move(q,3.0))
T2=r.fk('panda_hand'); print('hand',np.round(T2[:3,3],4))
f=r.fingers(); print('fingers',np.round(f,5),'gap',round(float(abs(f[0])+abs(f[1])),5))
r.snap('robot0_eye_in_hand','lift_eih.png'); r.snap('robot0_agentview_left','lift_avl.png'); r.snap('robot0_agentview_right','lift_avr.png')
from nav_msgs.msg import Odometry
m=r.grab('/odom',Odometry); p=m.pose.pose.position
print('odom',round(p.x,4),round(p.y,4))
print('T_w_l0 t',np.round(r.T_w_l0()[:3,3],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 46  ran 15.9s
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('lift2')
T=r.fk('panda_hand'); print('cur hand',np.round(T[:3,3],4))
a=T[:3,2]
opts=[('up05',np.array([0,0,0.05])),('up07',np.array([0,0,0.07])),
      ('back09',-a*0.09),('back06',-a*0.06),
      ('up05back05',np.array([0,0,0.05])-a*0.05),
      ('up04',np.array([0,0,0.04]))]
for nm,d in opts:
    Tn=T.copy(); Tn[:3,3]=T[:3,3]+d
    q=r.ik(Tn,seed=r.arm_q())
    print(nm, np.round(Tn[:3,3],4), 'IK', 'ok' if q is not None else 'FAIL', np.round(q,3) if q is not None else '')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 47  ran 49.5s
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('lift3')
q=[-1.391,-0.441,1.013,-1.198,0.58,1.781,0.626]
print('move', r.move(q,3.0))
T=r.fk('panda_hand'); print('hand',np.round(T[:3,3],4))
f=r.fingers(); print('fingers',np.round(f,5),'gap',round(float(abs(f[0])+abs(f[1])),5))
r.snap('robot0_eye_in_hand','lift_eih.png'); r.snap('robot0_agentview_left','lift_avl.png')
from nav_msgs.msg import Odometry
m=r.grab('/odom',Odometry); p=m.pose.pose.position
print('odom',round(p.x,4),round(p.y,4),'  T_w_l0',np.round(r.T_w_l0()[:3,3],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 48  ran 31.0s
timeout 300 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('mugpose')
cam='robot0_eye_in_hand'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
blue=cv2.inRange(hsv,(95,120,120),(115,255,255))>0
ok=np.isfinite(dep)&(dep>0)&(dep<0.5)
sel=blue&ok
Pp=Wc[:,sel].T
print('blue near pts',len(Pp))
print(' x',np.round([Pp[:,0].min(),Pp[:,0].max()],4),'y',np.round([Pp[:,1].min(),Pp[:,1].max()],4),'z',np.round([Pp[:,2].min(),Pp[:,2].max()],4))
zt=Pp[:,2].max()
rim=Pp[Pp[:,2]>zt-0.004]
def fit(Q):
    x,y=Q[:,0],Q[:,1]; A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; a_,b_=c[0]/2,c[1]/2
    return a_,b_,np.sqrt(c[2]+a_**2+b_**2)
a_,b_,R=fit(rim)
print(f'rim top z={zt:.4f} fit center=({a_:.4f},{b_:.4f}) R={R:.4f} n={len(rim)}')
Th=r.fk('panda_hand'); o=Th[:3,3]; ax=Th[:3,2]
tcp=o+ax*0.1034
print('TCP',np.round(tcp,4))
print('predicted rim center',np.round(tcp+np.array([-0.0395,0,0.022]),4))
print('measured  rim center',np.round([a_,b_,zt],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 49  ran 33.6s
timeout 120 python3 -u -c "
import numpy as np
Wc=np.load('cm_w.npy')
for v in range(240,266):
    row=[]
    for u in [305,315,322,330,340]:
        p=Wc[:,v,u]
        row.append(f'({u}) {p[1]:+.3f},{p[2]:.3f}')
    print(v,'  '.join(row))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 50  ran 89.6s
python3 - <<'EOF'
s=open('scene.py').read()
s=s.replace(""" 'coffee':    (1.185, 1.352, -0.465, -0.125, 0.90, 1.270),""",
""" 'cm_base':   (1.185, 1.352, -0.465, -0.125, 0.90, 0.9735),
 'cm_pedest': (1.185, 1.352, -0.330, -0.125, 0.9735, 1.108),
 'cm_head':   (1.225, 1.325, -0.430, -0.125, 1.108, 1.165),
 'cm_body':   (1.185, 1.352, -0.465, -0.125, 1.165, 1.270),""")
open('scene.py','w').write(s)
EOF
cat > place.py <<'EOF'
import numpy as np
from rb import Rb
import scene

# measured mug-in-gripper (world offsets while held at phi=0)
D_H  = np.array([-0.0403, -0.0049])   # rim centre minus TCP, horizontal
D_Z  = 0.0132                          # rim top minus TCP z
MUG_H = 0.0552
MUG_R = 0.047
TCPL = 0.1034

def rimpose(phi_deg):
    p=np.deg2rad(phi_deg)
    f=np.array([np.cos(p),np.sin(p),0.0])
    a=np.array([-np.sin(p),np.cos(p),0.0])*np.sqrt(0.5) + np.array([0,0,-np.sqrt(0.5)])
    hx=np.cross(f,a)
    return np.stack([hx,f,a],1), a

def hand_for_mug(mug_xy, mug_bottom_z, phi_deg):
    """hand 4x4 that puts the mug centre at mug_xy with its base at z."""
    R,a = rimpose(phi_deg)
    c,s = np.cos(np.deg2rad(phi_deg)), np.sin(np.deg2rad(phi_deg))
    dh = np.array([c*D_H[0]-s*D_H[1], s*D_H[0]+c*D_H[1]])
    tcp = np.array([mug_xy[0]-dh[0], mug_xy[1]-dh[1],
                    mug_bottom_z + MUG_H - D_Z])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=tcp - TCPL*a
    return T, tcp

def mug_pts(T):
    """world points of the held mug for collision checks."""
    R,o = T[:3,:3], T[:3,3]
    tcp = o + R[:,2]*TCPL
    phi = np.arctan2(R[1,1], R[0,1])       # finger axis yaw
    c,s = np.cos(phi), np.sin(phi)
    dh = np.array([c*D_H[0]-s*D_H[1], s*D_H[0]+c*D_H[1]])
    ctr = np.array([tcp[0]+dh[0], tcp[1]+dh[1], tcp[2]+D_Z])
    P=[]
    for dz in (0.0,-0.02,-MUG_H):
        for ang in range(0,360,30):
            A=np.deg2rad(ang)
            P.append(ctr+[MUG_R*np.cos(A), MUG_R*np.sin(A), dz])
        P.append(ctr+[0,0,dz])
    return np.array(P)

def full_pts(r,q):
    P,Ts = scene.pts_for(r,q)
    return np.vstack([P, mug_pts(Ts['panda_hand'])]), Ts

def check(r, qa, qb, n=9, margin=0.015, skip=(), label=''):
    qa=np.array(qa,float); qb=np.array(qb,float); bad=[]
    for i in range(n+1):
        q=qa+(qb-qa)*i/n
        P,_=full_pts(r,q)
        h=scene.hits(P,margin,skip)
        if h: bad.append((round(i/n,2),h))
    if bad:
        print(f'  !! {label}:')
        for t,h in bad: print(f'     t={t} {h}')
    else: print(f'  ok {label} clear (m={margin})')
    return not bad
EOF
timeout 1500 python3 -u -c "
import numpy as np
from rb import Rb
import place, scene
r=Rb('pl')
MUG=(1.2745,-0.395)
poses={
 'inter':  place.hand_for_mug(MUG,1.258,-90),
 'stage':  None,
 'insert': None,
 'down':   None,
}
Ti,tcpi=place.hand_for_mug(MUG,1.258,-90)
# staging: same x/z as insert but 17cm forward in y
Tins,tcpins=place.hand_for_mug(MUG,0.986,-90)
Tsg=Tins.copy(); Tsg[1,3]-=0.165
Tdn,_=place.hand_for_mug(MUG,0.978,-90)
Tint=Tsg.copy(); Tint[2,3]=Ti[2,3]
for nm,T in [('inter',Tint),('stage',Tsg),('insert',Tins),('down',Tdn)]:
    print(nm,'hand',np.round(T[:3,3],4))
qcur=r.arm_q()
qs={}
seed=qcur
for nm,T in [('inter',Tint),('stage',Tsg),('insert',Tins),('down',Tdn)]:
    q=r.ik(T,seed=seed)
    if q is None: print(nm,'IK FAIL'); continue
    qs[nm]=q; seed=q
    Tf=r.fk('panda_hand',q)
    P,_=place.full_pts(r,q)
    print(nm,'q',np.round(q,3),'poserr',round(float(np.linalg.norm(Tf[:3,3]-T[:3,3])),4),'hits',scene.hits(P,0.010))
np.save('qs.npy', np.array([qs[k] for k in ['inter','stage','insert','down'] if k in qs]))
print('saved', list(qs))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 51  ran 36.1s
cat > path.py <<'EOF'
import numpy as np
from rb import Rb
import scene, place

def wp_T(mug_xy, bottom, phi):
    T,_ = place.hand_for_mug(mug_xy, bottom, phi)
    return T

def build(r, wps, seed, step=0.025, maxdq=0.6, margin=0.010, skip=()):
    """wps: list of (mug_xy, bottom_z, phi). Returns joint list or None."""
    qs=[]; q=list(seed)
    prev=wps[0]
    for w in wps[1:]:
        p0=np.array([*prev[0],prev[1]]); p1=np.array([*w[0],w[1]])
        d=np.linalg.norm(p1-p0); dphi=abs(w[2]-prev[2])
        n=max(2,int(np.ceil(max(d/step, dphi/12.0))))
        for i in range(1,n+1):
            t=i/n
            mxy=(p0[0]+(p1[0]-p0[0])*t, p0[1]+(p1[1]-p0[1])*t)
            bz=p0[2]+(p1[2]-p0[2])*t
            ph=prev[2]+(w[2]-prev[2])*t
            T=wp_T(mxy,bz,ph)
            sol=r.ik(T,seed=q)
            if sol is None:
                print(f'  IK fail at mug={np.round(mxy,3)} z={bz:.3f} phi={ph:.1f}')
                return None
            dq=float(np.max(np.abs(np.array(sol)-np.array(q))))
            if dq>maxdq:
                print(f'  jump dq={dq:.2f} at mug={np.round(mxy,3)} z={bz:.3f} phi={ph:.1f}')
                return None
            P,_=place.full_pts(r,sol)
            h=scene.hits(P,margin,skip)
            if h:
                print(f'  hit {h} at mug={np.round(mxy,3)} z={bz:.3f} phi={ph:.1f}')
                return None
            q=sol; qs.append(sol)
        prev=w
    return qs
EOF
timeout 1800 python3 -u -c "
import numpy as np
from rb import Rb
import path, place
r=Rb('bp')
q0=r.arm_q()
T=r.fk('panda_hand'); tcp=T[:3,3]+T[:3,2]*0.1034
m0=(tcp[0]+place.D_H[0], tcp[1]+place.D_H[1]); b0=tcp[2]+place.D_Z-place.MUG_H
print('start mug',np.round(m0,4),'bottom',round(float(b0),4))
wps=[(m0,b0,0.0),
     ((1.20,-0.52),1.36,-45.0),
     ((1.2745,-0.56),1.26,-90.0),
     ((1.2745,-0.56),0.986,-90.0)]
qs=path.build(r,wps,q0)
print('segment A points:', None if qs is None else len(qs))
if qs is not None:
    np.save('pathA.npy',np.array(qs))
    print('last q',np.round(qs[-1],3))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 52  ran 145.4s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import place
r=Rb('exA')
qs=np.load('pathA.npy')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
viol=[(i,j,round(float(qs[i,j]),4)) for i in range(len(qs)) for j in range(7) if qs[i,j]<lim[j,0] or qs[i,j]>lim[j,1]]
print('limit violations:',viol[:10],'count',len(viol))
qs=np.clip(qs,lim[:,0]+1e-4,lim[:,1]-1e-4)
print('move ->', r.move(qs.tolist(), 12.0))
qa=r.arm_q(); print('reached',np.round(qa,4))
print('target ',np.round(qs[-1],4))
print('max err',round(float(np.max(np.abs(np.array(qa)-qs[-1]))),4))
T=r.fk('panda_hand'); tcp=T[:3,3]+T[:3,2]*0.1034
print('hand',np.round(T[:3,3],4),'TCP',np.round(tcp,4))
f=r.fingers(); print('fingers',np.round(f,5),'gap',round(float(abs(f[0])+abs(f[1])),5))
r.snap('robot0_agentview_left','stage_avl.png'); r.snap('robot0_agentview_right','stage_avr.png')
print('imgs saved')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 53  ran 23.8s
timeout 400 python3 -u -c "
import numpy as np, cv2
from rb import Rb
import place
r=Rb('vst')
T=r.fk('panda_hand'); tcp=T[:3,3]+T[:3,2]*0.1034
pred_ctr=np.array([tcp[0]+0.0049*0+(-0.0049), tcp[1]+0.0403, tcp[2]+place.D_Z])
# phi=-90: dh = Rz(-90)@D_H = (D_H[1], -D_H[0]) = (-0.0049, 0.0403)
print('TCP',np.round(tcp,4))
print('predicted rim ctr',np.round(pred_ctr,4),' predicted bottom',round(float(pred_ctr[2]-place.MUG_H),4))
allp=[]
for cam in ['robot0_agentview_left','robot0_agentview_right']:
    img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
    k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    Tc=r.tf('world',cam+'_optical_frame')
    H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
    P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
    Wc=np.einsum('ij,jhw->ihw',Tc,P)[:3]
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    m=(cv2.inRange(hsv,(95,120,120),(115,255,255))>0)&np.isfinite(dep)&(dep>0)
    Q=Wc[:,m].T
    if len(Q)<30: print(cam,'few pts',len(Q)); continue
    allp.append(Q)
    print(cam,'n',len(Q),'x',np.round([Q[:,0].min(),Q[:,0].max()],4),'y',np.round([Q[:,1].min(),Q[:,1].max()],4),'z',np.round([Q[:,2].min(),Q[:,2].max()],4))
Q=np.vstack(allp)
zt=Q[:,2].max(); rim=Q[Q[:,2]>zt-0.005]
x,y=rim[:,0],rim[:,1]; A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; a_,b_=c[0]/2,c[1]/2; R=np.sqrt(c[2]+a_**2+b_**2)
print(f'MEASURED rim top z={zt:.4f} ctr=({a_:.4f},{b_:.4f}) R={R:.4f} n={len(rim)}')
print('mug bottom (rim-0.0552)',round(float(zt-0.0552),4))
print('lowest blue z',round(float(Q[:,2].min()),4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 54  ran 25.5s
timeout 400 python3 -u -c "
import numpy as np, cv2
from rb import Rb
import place
r=Rb('vst2')
T=r.fk('panda_hand'); tcp=T[:3,3]+T[:3,2]*0.1034
pred=np.array([tcp[0]-0.0049, tcp[1]+0.0403, tcp[2]+place.D_Z])
allp=[]
for cam in ['robot0_agentview_left','robot0_agentview_right']:
    img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
    k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
    Tc=r.tf('world',cam+'_optical_frame')
    H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
    P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
    Wc=np.einsum('ij,jhw->ihw',Tc,P)[:3]
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    m=(cv2.inRange(hsv,(95,140,140),(112,255,255))>0)&np.isfinite(dep)&(dep>0)
    m&=(Wc[0]>1.18)&(Wc[0]<1.38)&(Wc[1]>-0.70)&(Wc[1]<-0.46)&(Wc[2]>0.95)&(Wc[2]<1.12)
    Q=Wc[:,m].T; allp.append(Q)
    print(cam,'n',len(Q))
    if len(Q): print('  x',np.round([Q[:,0].min(),Q[:,0].max()],4),'y',np.round([Q[:,1].min(),Q[:,1].max()],4),'z',np.round([Q[:,2].min(),Q[:,2].max()],4))
    # overlay predicted mug
    Ti=np.linalg.inv(Tc); vis=img.copy()
    def pr(p):
        q=Ti@np.array([*p,1.0]); return int(round(q[0]/q[2]*fx+cx)),int(round(q[1]/q[2]*fy+cy))
    for dz,col in [(0.0,(0,255,0)),(-0.0552,(0,0,255))]:
        for ang in range(0,360,5):
            A=np.deg2rad(ang); rr=0.0403 if dz==0 else 0.030
            u,v=pr(pred+[rr*np.cos(A),rr*np.sin(A),dz])
            if 0<=u<640 and 0<=v<480: cv2.circle(vis,(u,v),1,col,-1)
    cv2.imwrite(cam.split('_')[-1]+'_ov.png',vis)
Q=np.vstack([a for a in allp if len(a)])
zt=Q[:,2].max(); rim=Q[Q[:,2]>zt-0.006]
x,y=rim[:,0],rim[:,1]; A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; a_,b_=c[0]/2,c[1]/2; R=np.sqrt(c[2]+a_**2+b_**2)
print('predicted rim ctr',np.round(pred,4))
print(f'MEASURED rim top z={zt:.4f} ctr=({a_:.4f},{b_:.4f}) R={R:.4f} n={len(rim)}')
print('implied bottom',round(float(zt-0.0552),4),' predicted bottom',round(float(pred[2]-0.0552),4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 55  ran 12.3s
python3 -c "
import cv2
for n in ['right_ov','left_ov']:
    img=cv2.imread(n+'.png')
    if n=='right_ov': c=img[270:330, 230:300]
    else: c=img[270:340, 370:450]
    cv2.imwrite(n+'_z.png', cv2.resize(c,None,fx=9,fy=9,interpolation=cv2.INTER_NEAREST))
print('ok')"

# openrua op 56  ran 56.5s
timeout 1800 python3 -u -c "
import numpy as np
from rb import Rb
import path, place, scene
from geometry_msgs.msg import WrenchStamped
r=Rb('bpB')
w=r.grab('/franka_robot_state_broadcaster/external_wrench',WrenchStamped)
print('wrench baseline f=',np.round([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z],3))
q0=r.arm_q()
wps=[((1.2742,-0.560),0.9857,-90.0),
     ((1.2742,-0.560),0.9900,-90.0),
     ((1.2745,-0.395),0.9900,-90.0)]
qs=path.build(r,wps,q0,step=0.02,maxdq=0.4,margin=0.008)
print('segB pts',None if qs is None else len(qs))
if qs is not None:
    np.save('pathB.npy',np.array(qs))
    for i in (0,len(qs)//2,len(qs)-1):
        T=r.fk('panda_hand',qs[i]); print(' ',i,'hand',np.round(T[:3,3],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 57  ran 80.6s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import place
from geometry_msgs.msg import WrenchStamped
r=Rb('exB')
qs=np.load('pathB.npy')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
qs=np.clip(qs,lim[:,0]+1e-4,lim[:,1]-1e-4)
print('move ->', r.move(qs.tolist(), 6.0))
qa=r.arm_q(); print('max err',round(float(np.max(np.abs(np.array(qa)-qs[-1]))),4))
T=r.fk('panda_hand'); tcp=T[:3,3]+T[:3,2]*0.1034
print('hand',np.round(T[:3,3],4),'TCP',np.round(tcp,4))
print('mug ctr',np.round([tcp[0]-0.0049,tcp[1]+0.0403],4),'bottom',round(float(tcp[2]+place.D_Z-place.MUG_H),4))
f=r.fingers(); print('fingers',np.round(f,5),'gap',round(float(abs(f[0])+abs(f[1])),5))
w=r.grab('/franka_robot_state_broadcaster/external_wrench',WrenchStamped)
print('wrench',np.round([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z],3))
r.snap('robot0_agentview_right','ins_avr.png'); r.snap('robot0_eye_in_hand','ins_eih.png')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 58  ran 91.6s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import place
from geometry_msgs.msg import WrenchStamped
r=Rb('down')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
def wr():
    w=r.grab('/franka_robot_state_broadcaster/external_wrench',WrenchStamped)
    return np.array([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z])
base=wr(); print('base',np.round(base,3))
for bz in [0.9855,0.9815,0.9775,0.9745]:
    T,tcp=place.hand_for_mug((1.2745,-0.3958),bz,-90)
    q=r.ik(T,seed=r.arm_q())
    if q is None: print(bz,'IK FAIL'); break
    q=np.clip(q,lim[:,0]+1e-4,lim[:,1]-1e-4)
    r.move(q.tolist(),1.5)
    Tf=r.fk('panda_hand'); t=Tf[:3,3]+Tf[:3,2]*0.1034
    f=wr()
    print(f'bottom target {bz}: actual {t[2]+place.D_Z-place.MUG_H:.4f} wrench {np.round(f,3)} d={np.round(f-base,3)}')
    if abs(f[2]-base[2])>1.5: print('  CONTACT detected'); break
" 2>&1 | grep -v IP_LOCATOR

# openrua op 59  ran 88.3s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import place, scene
r=Rb('rel')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
print('open ->', r.gripper(0.04))
print('fingers',np.round(r.fingers(),5))
T=r.fk('panda_hand')
# lift straight up 0.055 keeping orientation, then retract -y
seq=[]
q=r.arm_q()
for dz in np.arange(0.008,0.056,0.008):
    Tn=T.copy(); Tn[2,3]=T[2,3]+dz
    s=r.ik(Tn,seed=q)
    if s is None: print('lift IK fail at',dz); break
    q=s; seq.append(s)
T2=r.fk('panda_hand',q)
for dy in np.arange(0.02,0.19,0.02):
    Tn=T2.copy(); Tn[1,3]=T2[1,3]-dy
    s=r.ik(Tn,seed=q)
    if s is None: print('retract IK fail at',dy); break
    q=s; seq.append(s)
print('seq',len(seq))
bad=0
for s in seq:
    P,_=scene.pts_for(r,s)
    h=scene.hits(P,0.008)
    if h: bad+=1; print(' hit',h)
print('bad',bad)
seq=np.clip(np.array(seq),lim[:,0]+1e-4,lim[:,1]-1e-4)
print('move ->', r.move(seq.tolist(), 6.0))
Tf=r.fk('panda_hand'); print('hand',np.round(Tf[:3,3],4))
r.snap('robot0_agentview_right','placed_avr.png'); r.snap('robot0_agentview_left','placed_avl.png')
print('done')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 60  ran 92.9s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
from look import hand_T_for_view
import scene
r=Rb('ver')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
T=hand_T_for_view((1.27,-0.80,1.45),(1.27,-0.38,1.03))
q=r.ik(T,seed=r.arm_q())
print('ik',None if q is None else np.round(q,3))
P,_=scene.pts_for(r,q); print('hits',scene.hits(P,0.01))
print('move',r.move(np.clip(q,lim[:,0]+1e-4,lim[:,1]-1e-4).tolist(),5.0))
r.snap('robot0_eye_in_hand','check1.png')
r.snap('robot0_agentview_right','check1_avr.png')
print('ok')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 61  ran 19.8s
timeout 400 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('meas')
cam='robot0_eye_in_hand'
img=r.snap(cam); dep=r.depth(cam).astype(np.float32)
k=r.info(cam).k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world',cam+'_optical_frame')
H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
np.save('final_w.npy',Wc); cv2.imwrite('final_rgb.png',img)
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
blue=cv2.inRange(hsv,(95,120,120),(115,255,255))>0
dark=cv2.inRange(hsv,(115,80,20),(150,255,110))>0
ok=np.isfinite(dep)&(dep>0)
m=(blue)&ok
Q=Wc[:,m].T
print('mug blue pts',len(Q))
print(' x',np.round([Q[:,0].min(),Q[:,0].max()],4),'y',np.round([Q[:,1].min(),Q[:,1].max()],4),'z',np.round([Q[:,2].min(),Q[:,2].max()],4))
zt=Q[:,2].max(); rim=Q[Q[:,2]>zt-0.004]
x,y=rim[:,0],rim[:,1]; A=np.stack([x,y,np.ones_like(x)],1); b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; a_,b_=c[0]/2,c[1]/2; R=np.sqrt(c[2]+a_**2+b_**2)
print(f'rim top z={zt:.4f} ctr=({a_:.4f},{b_:.4f}) R={R:.4f} n={len(rim)}')
print('implied mug bottom',round(float(zt-0.0552),4),'  tray z 0.9741')
# button pixels
n,lab,st,cen=cv2.connectedComponentsWithStats((dark&ok).astype('uint8'),8)
for i in range(1,n):
    if st[i,4]<12 or st[i,4]>400: continue
    u,v=int(cen[i][0]),int(cen[i][1])
    p=Wc[:,v,u]
    if not np.isfinite(p).all(): continue
    if p[2]>1.14 and p[1]<-0.40:
        print('dark blob',st[i,:5],'px',u,v,'world',np.round(p,4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 62  ran 10.9s
python3 -c "
import cv2
img=cv2.imread('final_rgb.png')
cv2.imwrite('btn2.png', cv2.resize(img[140:230, 280:370], None, fx=8, fy=8, interpolation=cv2.INTER_CUBIC))
"

# openrua op 63  ran 13.6s
timeout 200 python3 -u -c "
import numpy as np, cv2
Wc=np.load('final_w.npy'); img=cv2.imread('final_rgb.png')
for lbl,(u0,v0) in [('upper',(323,173)),('lower',(322,205))]:
    pts=[]
    for du in range(-4,5):
        for dv in range(-3,4):
            p=Wc[:,v0+dv,u0+du]
            if np.isfinite(p).all(): pts.append(p)
    P=np.array(pts)
    print(lbl,'center px',u0,v0,'world',np.round(Wc[:,v0,u0],4))
    print('   patch mean',np.round(P.mean(0),4),' y range',np.round([P[:,1].min(),P[:,1].max()],4),' z range',np.round([P[:,2].min(),P[:,2].max()],4),' x range',np.round([P[:,0].min(),P[:,0].max()],4))
# surrounding face for comparison (is the button proud of the surface?)
for lbl,(u0,v0) in [('face-left-of-upper',(305,173)),('face-right-of-upper',(341,173)),('face-above',(323,160))]:
    print(lbl, np.round(Wc[:,v0,u0],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 64  ran 100.0s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import scene
r=Rb('press')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
print('close ->', r.gripper(0.0)); print('fingers',np.round(r.fingers(),5))
R=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])   # approach = +y
def T_at(tcp):
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=np.array(tcp)-0.1034*R[:,2]; return T
BTN=np.array([1.2742,-0.4551,1.2268])
q=r.arm_q(); seq=[]
for dy in [-0.10,-0.08,-0.06,-0.05]:
    T=T_at(BTN+np.array([0,dy,0]))
    s=r.ik(T,seed=q)
    if s is None: print('IK fail at dy',dy); break
    P,_=scene.pts_for(r,s); h=scene.hits(P,0.010)
    if h: print('hit at dy',dy,h); break
    q=s; seq.append(s)
print('approach pts',len(seq))
if len(seq)==4:
    print('move ->', r.move(np.clip(np.array(seq),lim[:,0]+1e-4,lim[:,1]-1e-4).tolist(),5.0))
    T=r.fk('panda_hand'); print('hand',np.round(T[:3,3],4),'TCP',np.round(T[:3,3]+T[:3,2]*0.1034,4))
    r.snap('robot0_eye_in_hand','before_press.png'); r.snap('robot0_agentview_right','before_press_avr.png')
    np.save('press_seed.npy',np.array(r.arm_q()))
    print('staged')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 65  ran 49.9s
timeout 600 python3 -u -c "
import numpy as np
from rb import Rb
r=Rb('fchk')
print('fingers now', np.round(r.fingers(),5))
r.gripper(0.0)
print('after re-close', np.round(r.fingers(),5))
Th=r.fk('panda_hand')
print('hand R\n',np.round(Th[:3,:3],3))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 66  ran 187.5s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
from geometry_msgs.msg import WrenchStamped
r=Rb('press2')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
R=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
def T_at(tcp):
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=np.array(tcp)-0.1034*R[:,2]; return T
BTN=np.array([1.2742,-0.4551,1.2268])
def wr():
    w=r.grab('/franka_robot_state_broadcaster/external_wrench',WrenchStamped)
    return np.array([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z])
r.snap('robot0_eye_in_hand','before_press.png')
r.snap('robot0_agentview_right','before_avr.png')
base=wr(); print('base wrench',np.round(base,3))
q=r.arm_q()
for y in [-0.490,-0.475,-0.465,-0.458,-0.452,-0.446,-0.440]:
    T=T_at(np.array([BTN[0],y,BTN[2]]))
    s=r.ik(T,seed=q)
    if s is None: print('IK fail y',y); break
    s=np.clip(s,lim[:,0]+1e-4,lim[:,1]-1e-4)
    code=r.move(s.tolist(),1.5); q=r.arm_q()
    Tf=r.fk('panda_hand'); tcp=Tf[:3,3]+Tf[:3,2]*0.1034
    f=wr()
    print(f'y={y}  tcp_y={tcp[1]:.4f} code={code} wrench={np.round(f,2)} d={np.round(f-base,2)}')
r.snap('robot0_eye_in_hand','after_press.png')
r.snap('robot0_agentview_right','after_avr.png')
print('done')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 67  ran 91.7s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
from geometry_msgs.msg import WrenchStamped
r=Rb('back')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
R=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
BTN=np.array([1.2742,-0.4551,1.2268])
q=r.arm_q()
for y in [-0.47,-0.50,-0.53]:
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=np.array([BTN[0],y,BTN[2]])-0.1034*R[:,2]
    s=r.ik(T,seed=q)
    if s is None: break
    s=np.clip(s,lim[:,0]+1e-4,lim[:,1]-1e-4); r.move(s.tolist(),1.5); q=r.arm_q()
w=r.grab('/franka_robot_state_broadcaster/external_wrench',WrenchStamped)
print('wrench after retract',np.round([w.wrench.force.x,w.wrench.force.y,w.wrench.force.z],3))
T=r.fk('panda_hand'); print('TCP',np.round(T[:3,3]+T[:3,2]*0.1034,4))
r.snap('robot0_eye_in_hand','post_press.png'); r.snap('robot0_agentview_right','post_avr.png')
print('ok')
" 2>&1 | grep -v IP_LOCATOR
python3 -c "
import cv2, numpy as np
b=cv2.imread('before_press.png'); a=cv2.imread('post_press.png')
d=cv2.absdiff(b,a); print('mean abs diff', float(d.mean()), 'max', int(d.max()))
ys,xs=np.nonzero(d.max(2)>25)
print('changed px', len(xs), 'bbox', (xs.min(),ys.min(),xs.max(),ys.max()) if len(xs) else None)
cv2.imwrite('diff.png', cv2.resize(d,None,fx=1,fy=1))
"

# openrua op 68  ran 84.7s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
r=Rb('cmp')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
q=[0.012,-1.649,-0.071,-2.392,-0.068,1.53,0.751]
print('move',r.move(np.clip(q,lim[:,0]+1e-4,lim[:,1]-1e-4).tolist(),5.0))
r.snap('robot0_eye_in_hand','check2.png')
import cv2
dep=r.depth('robot0_eye_in_hand').astype(np.float32)
k=r.info('robot0_eye_in_hand').k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
T=r.tf('world','robot0_eye_in_hand_optical_frame')
H,W=dep.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*dep/fx,(vs-cy)*dep/fy,dep,np.ones_like(dep)])
Wc=np.einsum('ij,jhw->ihw',T,P)[:3]
np.save('final_w2.npy',Wc)
print('cam T match?', np.round(T[:3,3],4))
for lbl,(u,v) in [('upper-btn',(323,173)),('lower-btn',(322,205)),('face',(305,173))]:
    print(lbl, np.round(Wc[:,v,u],4))
" 2>&1 | grep -v IP_LOCATOR
python3 -c "
import cv2, numpy as np
b=cv2.imread('final_rgb.png'); a=cv2.imread('check2.png')
d=cv2.absdiff(b,a)
print('mean',round(float(d.mean()),3),'max',int(d.max()))
m=d.max(2)>25; ys,xs=np.nonzero(m)
print('changed px',len(xs))
if len(xs): print('bbox',xs.min(),ys.min(),xs.max(),ys.max())
cv2.imwrite('diff2.png', np.hstack([b,a]))
"

# openrua op 69  ran 18.1s
timeout 400 python3 -u -c "
import numpy as np, cv2
from rb import Rb
r=Rb('btnchk')
img=r.snap('robot0_eye_in_hand'); cv2.imwrite('check2.png',img)
Wc=np.load('final_w2.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
dark=(cv2.inRange(hsv,(0,0,0),(180,255,90))>0)
n,lab,st,cen=cv2.connectedComponentsWithStats(dark.astype('uint8'),8)
cands=[]
for i in range(1,n):
    x,y,w,h,a=st[i]
    if not (15<a<400): continue
    u,v=int(cen[i][0]),int(cen[i][1])
    p=Wc[:,v,u]
    if not np.isfinite(p).all(): continue
    if 1.14<p[2]<1.28 and p[1]<-0.35 and 1.2<p[0]<1.36:
        cands.append((a,u,v,np.round(p,4),(x,y,w,h)))
for c in sorted(cands): print('dark blob area',c[0],'px',c[1],c[2],'world',c[3],'bbox',c[4])
print()
# protrusion: compare button y to surrounding face y at same height
for (a,u,v,p,bb) in sorted(cands):
    fy=[]
    for du in (-20,-14,14,20):
        q=Wc[:,v,u+du]
        if np.isfinite(q).all(): fy.append(float(q[1]))
    if fy: print(f'blob px({u},{v}) btn_y={p[1]:.4f} face_y={np.mean(fy):.4f} protrusion={np.mean(fy)-p[1]:.4f}')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 70  ran 41.6s
python3 -c "
import cv2, numpy as np
a=cv2.imread('final_rgb.png'); b=cv2.imread('check2.png')
# mug interior region differs slightly in position between frames; crop generously
ca=a[225:280, 285:345]; cb=b[215:275, 270:335]
cv2.imwrite('mug_ba.png', np.hstack([cv2.resize(ca,(360,330),interpolation=cv2.INTER_NEAREST), cv2.resize(cb,(360,330),interpolation=cv2.INTER_NEAREST)]))
print('ok')"

# openrua op 71  ran 75.3s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
import scene
r=Rb('fin')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
q=[0.0,-1.60,0.0,-2.60,0.0,1.50,0.785]
P,_=scene.pts_for(r,q); print('stow hits',scene.hits(P,0.01))
print('move',r.move(np.clip(q,lim[:,0]+1e-4,lim[:,1]-1e-4).tolist(),5.0))
T=r.fk('panda_hand'); print('hand',np.round(T[:3,3],4))
r.snap('robot0_agentview_right','FINAL_avr.png'); r.snap('robot0_agentview_left','FINAL_avl.png')
r.snap('robot0_eye_in_hand','FINAL_eih.png')
print('done')
" 2>&1 | grep -v IP_LOCATOR

# openrua op 72  ran 65.1s
timeout 1800 python3 -u look.py 1.27 -0.72 1.32 1.272 -0.39 1.01 --exec --secs 4 --tag FINAL_look 2>&1 | grep -v IP_LOCATOR
python3 -c "
import cv2
img=cv2.imread('FINAL_look.png'); print(img.shape)
cv2.imwrite('FINAL_mug.png', cv2.resize(img[150:330,230:410],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 73  ran 74.6s
timeout 1800 python3 -u -c "
import numpy as np, yaml
from rb import Rb
r=Rb('stow')
lim=np.array(yaml.safe_load(open('machine.yaml'))['actuators'][0]['limits_rad'])
q=[0.0,-1.60,0.0,-2.60,0.0,1.50,0.785]
print('move',r.move(np.clip(q,lim[:,0]+1e-4,lim[:,1]-1e-4).tolist(),5.0))
print('final q',np.round(r.arm_q(),4))
print('fingers',np.round(r.fingers(),5))
T=r.fk('panda_hand'); print('hand',np.round(T[:3,3],4))
" 2>&1 | grep -v IP_LOCATOR

# openrua op 74  ran 9.3s
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/compute-ik-targets-link8.md)"
cat > /claude-config/projects/-workspace/memory/compute-ik-targets-link8.md <<'OPENRUA_EOF'
---
name: compute-ik-targets-link8
description: On this PandaOmron workstation /compute_ik solves for panda_link8, not panda_hand — a 45° roll offset the shipped docs get wrong
metadata:
  type: project
---

The `/compute_ik` service on this machine solves for **panda_link8**, even
though `tools/action/ik_move.py` documents its pose argument as "where the
HAND frame goes". `panda_hand = panda_link8 * Rz(-45°)`, so a requested
pose comes out with the finger axis rotated 45° from what you asked for.
Position is unaffected (link8 and hand share an origin), which is why the
error is easy to miss — FK position matches, orientation silently doesn't.

**Why:** cost an hour of bogus IK-failure sweeps; grasp planning that
depends on the finger-closing direction (rim pinches, button presses) is
wrong by 45° without it.

**How to apply:** to command a `panda_hand` pose `H`, send
`H @ Rz(+45°)` to `/compute_ik`. Verify with `/compute_fk` on
`panda_hand` and compare the full rotation matrix, not just position.

Related: [[panda-tf-arm-chain-disconnected]]
OPENRUA_EOF

# openrua op 75  ran 2.4s
ls /claude-config/projects/-workspace/memory/ 2>/dev/null; echo "---"; cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null || echo "(no MEMORY.md)"

# openrua op 76  ran 2.7s
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [compute_ik targets panda_link8](compute-ik-targets-link8.md) — IK poses are link8, not panda_hand; 45° roll offset the workspace docs get wrong.
OPENRUA_EOF
