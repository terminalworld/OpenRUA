#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; mkdir -p snaps; for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait; ls -la snaps; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; python3 -c "
import cv2
im=cv2.imread('snaps/birdview.png'); crop=im[130:320,180:400]; cv2.imwrite('snaps/bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/agentview.png'); crop=im[140:400,100:470]; cv2.imwrite('snaps/agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
cd /workspace; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | head -60; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 7
cd /workspace; cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfd'); b=Buffer(); TransformListener(b,n)
for _ in range(30): rclpy.spin_once(n, timeout_sec=0.2)
print(b.all_frames_as_string())
for f in ['panda_link0','agentview_optical_frame','birdview_optical_frame','frontview_optical_frame','sideview_optical_frame','robot0_eye_in_hand_optical_frame','robot0_robotview_optical_frame','panda_hand']:
    try:
        t=b.lookup_transform('world',f,rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f, [round(v,4) for v in (tr.x,tr.y,tr.z)], [round(v,4) for v in (q.x,q.y,q.z,q.w)])
    except Exception as e: print(f,'ERR',e)
EOF
timeout 60 python3 tfdump.py

# openrua op 8
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/bird_depth.png; ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 9
cd /workspace; cat > cloud.py <<'EOF'
import numpy as np, sys
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical frame: quat (0.7071,0.7071,0,0) => R = rot about (1,1,0)/sqrt2 by 180deg
q=np.array([0.7071,0.7071,0.0,0.0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([-0.2,0,3.0])
def w(u,v):
    Z=d[v,u]; p=np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z]); return R@p+T
print('R=',R)
for name,(u,v) in {'table':(320,400),'book':(325,272),'caddy_back_right':(280,172),'caddy_front_right':(280,196),'caddy_left':(237,185),'caddy_top_wall_left':(217,185),'mug':(360,240)}.items():
    print(name,(u,v),d[v,u],w(u,v).round(4))
# table height stats
print('table z hist near center', np.round(w(300,350),3), np.round(w(500,300),3))
EOF
python3 cloud.py

# openrua op 10
cd /workspace; cat > cloud2.py <<'EOF'
import numpy as np
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2   # world x  (image down)
Y=(uu-cx)*d/fx         # world y  (image right)
Z=3.0-d
# objects above table
mask=(Z>0.885)&(X>-0.6)&(X<0.4)&(np.abs(Y)<0.6)
# exclude robot: robot occupies X<-0.3ish and Y in (-0.1,0.2) at high Z; print clusters by Z
import collections
# book: region around (325,272)
bm=mask&(uu>300)&(uu<350)&(vv>255)&(vv<295)
print('book pts',bm.sum(),'X',X[bm].min().round(3),X[bm].max().round(3),'Y',Y[bm].min().round(3),Y[bm].max().round(3),'Z',Z[bm].min().round(3),Z[bm].max().round(3))
# book top-surface points (Z>1.0)
bt=bm&(Z>1.0)
pts=np.stack([X[bt],Y[bt]],1); print('book top pts',len(pts), 'center',pts.mean(0).round(4))
# PCA for orientation
c=pts-pts.mean(0); u,s,vt=np.linalg.svd(c,full_matrices=False); print('book axis',vt[0].round(3),'extent along',(c@vt[0]).min().round(3),(c@vt[0]).max().round(3),'across',(c@vt[1]).min().round(3),(c@vt[1]).max().round(3))
# caddy: region uu 210-305, vv 155-210
cm=mask&(uu>205)&(uu<310)&(vv>150)&(vv<215)
print('caddy Z range',Z[cm].min().round(3),Z[cm].max().round(3))
# walls: Z > 0.95
wm=cm&(Z>0.95)
print('caddy walls X',X[wm].min().round(3),X[wm].max().round(3),'Y',Y[wm].min().round(3),Y[wm].max().round(3))
# histogram of wall heights
hs,edges=np.histogram(Z[wm],bins=20); 
for h,e in zip(hs,edges): print(round(e,3),h)
# print an ascii map of Z over caddy region at 2px stride
for v in range(150,215,3):
    row=''
    for u in range(205,320,2):
        z=Z[v,u]
        row+= '#' if z>1.15 else ('W' if z>0.98 else ('w' if z>0.93 else ('f' if z>0.89 else '.')))
    print(v,row)
EOF
python3 cloud2.py

# openrua op 11
cd /workspace; cat > cloud3.py <<'EOF'
import numpy as np
d=np.load('snaps/bird_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2; Y=(uu-cx)*d/fx; Z=3.0-d
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print('back wall row v=162..164 Z:', Z[162:165,215:300].max(0)[::5])
print('front wall rows v=199..203 Z:', Z[199:204,215:300].max(0)[::5])
print('divider row v=183 Z:', Z[182:185,260:300].max(0)[::4])
print('left wall Z:', Z[165:200,211:216].max(1)[::5])
print('mid divider u=257-259 Z:', Z[165:200,256:261].max(1)[::5])
print('X of back wall', X[162,250], 'X of front wall', X[201,250], 'X divider', X[183,280])
print('Y left wall', Y[180,213], 'Y mid divider', Y[180,258])
print('floor Z in back-right', Z[170:180,265:295].mean(), 'front-right', Z[187:197,265:295].mean(), 'left', Z[170:195,220:250].mean())
print('mug region: Z max', Z[225:260,340:385].max(), 'at', np.unravel_index(Z[225:260,340:385].argmax(), (35,45)))
EOF
python3 cloud3.py

# openrua op 12
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw snaps/agent_depth.png; ros2 topic echo /agentview/color/camera_info --once | grep -A3 "^k:"
cat > cloudgen.py <<'EOF'
import numpy as np
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(npy, fx, fy, cx, cy, t, q):
    d=np.load(npy); H,W=d.shape
    vv,uu=np.mgrid[0:H,0:W]
    P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
    R=quat_R(*q)
    return P@R.T+np.array(t), d
EOF
cat > agentcloud.py <<'EOF'
import numpy as np, sys
from cloudgen import cloud
fx=float(sys.argv[1])
Pw,d=cloud('snaps/agent_depth.npy',fx,fx,320,240,[0.4586,0.0,1.6104],[0.638,0.638,-0.3048,-0.3048])
X,Y,Z=Pw[...,0],Pw[...,1],Pw[...,2]
np.set_printoptions(precision=3,suppress=True,linewidth=200)
print('table sample', Pw[420,320], Pw[300,100], Pw[300,600])
# caddy: points with Z>0.93 and X in -0.5..-0.3
m=(Z>0.93)&(Z<1.1)&(X>-0.5)&(X<-0.3)
print('caddy pts',m.sum(),'X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max(),'Z',Z[m].max())
# wall top segments: Z>1.03
for lo,hi in [(1.03,1.1),(0.94,1.0)]:
    mm=m&(Z>lo)&(Z<hi)
    print('walls Z in',lo,hi,'n',mm.sum(),'X',X[mm].min().round(3),X[mm].max().round(3),'Y',Y[mm].min().round(3),Y[mm].max().round(3))
# Y histogram of tall wall points
mm=m&(Z>1.03)
h,e=np.histogram(Y[mm],bins=np.arange(-0.42,0.06,0.02))
for a,b in zip(e,h): print(round(a,2),b)
# book
bm=(Z>0.95)&(X>-0.2)&(X<0.0)&(Y>-0.1)&(Y<0.15)
print('book X',X[bm].min(),X[bm].max(),'Y',Y[bm].min(),Y[bm].max(),'Zmax',Z[bm].max())
EOF
python3 agentcloud.py $(ros2 topic echo /agentview/color/camera_info --once | grep -A1 "^k:" | tail -1 | tr -d ' -')

# openrua op 13
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:" | tr '\n' ' '; echo
cat > eihcloud.py <<'EOF'
import numpy as np, sys
from cloudgen import cloud
fx=float(sys.argv[1]); cx=float(sys.argv[2]); cy=float(sys.argv[3])
Pw,d=cloud('snaps/eih_depth.npy',fx,fx,cx,cy,[-0.243,0.0,1.2673],[0.707,-0.7066,-0.0201,0.0201])
X,Y,Z=Pw[...,0],Pw[...,1],Pw[...,2]
np.set_printoptions(precision=3,suppress=True,linewidth=250)
print('corners', Pw[0,0],Pw[0,639],Pw[479,0],Pw[479,639])
print('table sample', Pw[200,320])
m=(Z>1.03)&(Z<1.1)&(X>-0.5)&(X<-0.3)
print('tall walls X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max())
h,e=np.histogram(Y[m],bins=np.arange(-0.42,0.12,0.02))
for a,b in zip(e,h): print(round(a,2),b)
# for Y>-0.07 region, X histogram of tall walls
mm=m&(Y>-0.06)&(Y<0.06)
h,e=np.histogram(X[mm],bins=np.arange(-0.48,-0.30,0.01))
print('right column tall walls X hist'); 
for a,b in zip(e,h): print(round(a,2),b)
mm=(Z>0.93)&(Z<1.03)&(X>-0.5)&(X<-0.3)&(Y>-0.06)&(Y<0.06)
h,e=np.histogram(X[mm],bins=np.arange(-0.48,-0.30,0.01))
print('right column low walls X hist'); 
for a,b in zip(e,h): print(round(a,2),b)
# ascii map of Z on grid X -0.48..-0.30 (rows), Y -0.40..0.10 (cols), 1cm
mask=(X>-0.5)&(X<-0.28)&(Y>-0.42)&(Y<0.12)
xi=((X-(-0.48))/0.01).astype(int); yi=((Y-(-0.40))/0.01).astype(int)
grid=np.full((19,53),np.nan)
for a,b,z in zip(xi[mask],yi[mask],Z[mask]):
    if 0<=a<19 and 0<=b<53: grid[a,b]=np.nanmax([grid[a,b],z])
for r in range(19):
    print(f'{-0.48+r*0.01:+.2f} '+''.join('?' if np.isnan(z) else ('T' if z>1.03 else ('t' if z>0.98 else ('l' if z>0.93 else ('f' if z>0.89 else '.')))) for z in grid[r]))
EOF
python3 eihcloud.py $(ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:" | tail -9 | tr -d ' -' | sed -n '1p;3p;6p' | tr '\n' ' ')

# openrua op 14
cd /workspace; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; cat > fkcheck.py <<'EOF'
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
arm=[f'panda_joint{i}' for i in range(1,8)]
for nm,p in zip(js['m'].name,js['m'].position):
    if nm in arm: req.robot_state.joint_state.name.append(nm); req.robot_state.joint_state.position.append(p)
req.header.frame_id=''
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code, r.pose_stamped[0].header.frame_id, r.pose_stamped[0].pose)
EOF
timeout 60 python3 fkcheck.py

# openrua op 15
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
"""Small helper layer over the manifest ports: joints, FK, IK, trajectory, gripper."""
import math
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(theta):
    """Hand pointing straight down (hand Z = world -Z), yawed by theta about world Z.
    theta=0: hand X = world X, hand Y = world -Y."""
    return (math.cos(theta / 2), math.sin(theta / 2), 0.0, 0.0)


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node(name)
        self._js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   self._on_js, 1)
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while "m" not in self._js:
            rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        js = self.joints()
        s = JointState()
        for j in ARM:
            s.name.append(j); s.position.append(js[j])
        return s

    def fk_pose(self, positions=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        if positions is None:
            req.robot_state.joint_state = self.arm_state()
        else:
            req.robot_state.joint_state.name = list(ARM)
            req.robot_state.joint_state.position = [float(p) for p in positions]
        f = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        r = f.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def tcp_from_hand(self, pos, quat):
        R = quat_R(*quat)
        return np.asarray(pos) + TCP * R[:, 2]

    def ik_solve(self, pos, quat, at_tcp=True, seed=None):
        """Return arm joint positions (manifest order) or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.avoid_collisions = False
        f = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            print("IK failed:", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [positions]
        for i, wp in enumerate(wps, 1):
            t = seconds * i / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        f = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, at_tcp=True):
        sol = self.ik_solve(pos, quat, at_tcp=at_tcp)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        hp, hq = self.fk_pose()
        tcp = self.tcp_from_hand(hp, hq)
        print("hand", hp.round(4), "tcp", tcp.round(4), "quat", np.round(hq, 4))
        return tcp

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={js.get('panda_finger_joint1'):.4f},{js.get('panda_finger_joint2'):.4f}")
        return js
OPENRUA_EOF

# openrua op 16
cd /workspace; cat > step1.py <<'EOF'
import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step1")
print("joints", {k: round(v,3) for k,v in r.joints().items()})
r.gripper(0.04)   # open
theta = math.radians(-40.5)
q = yaw_down_quat(theta)
R = quat_R(*q); print("hand X", R[:,0].round(3), "hand Y", R[:,1].round(3), "hand Z", R[:,2].round(3))
target = np.array([-0.092, 0.023, 1.20])
sol = r.ik_solve(target, q)
print("sol", None if sol is None else np.round(sol,3))
if sol:
    hp,hq = r.fk_pose(sol); print("FK hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), np.round(hq,4))
    r.move_joints(sol, 4.0)
    hp,hq = r.fk_pose(); print("now hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), np.round(hq,4))
EOF
timeout 600 python3 -u step1.py

# openrua op 17
cd /workspace; cat > iktest.py <<'EOF'
import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("iktest")
target = np.array([-0.092, 0.023, 1.20])
for deg in [0, -20, -40.5, -60, -90, 45, 90]:
    q = yaw_down_quat(math.radians(deg))
    sol = r.ik_solve(target, q)
    if sol:
        hp,hq = r.fk_pose(sol)
        R=quat_R(*hq)
        print(deg, "req q", np.round(q,3), "-> FK q", np.round(hq,3), "handX", R[:,0].round(3), "tcp", r.tcp_from_hand(hp,hq).round(3), "j7", round(sol[6],3))
EOF
timeout 300 python3 -u iktest.py

# openrua op 18
cd /workspace; timeout 30 ros2 run tf2_ros tf2_echo panda_hand panda_leftfinger 2>&1 | grep -A1 Translation | head -2; timeout 30 ros2 run tf2_ros tf2_echo panda_link8 panda_hand 2>&1 | grep -E "Translation|Rotation: in Quat" | head -2

# openrua op 19
cd /workspace; cat > step2.py <<'EOF'
import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step2")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
tcp = r.move_tcp([-0.092, 0.023, 1.20], q, 3.0)
hp,hq = r.fk_pose(); R=quat_R(*hq); print("handX", R[:,0].round(3), "handY(close dir)", R[:,1].round(3))
EOF
timeout 600 python3 -u step2.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 20
cd /workspace; cat > check.py <<'EOF'
import numpy as np
from robot import Robot, quat_R
r = Robot("check")
js = r.joints(); print({k: round(v,3) for k,v in js.items()})
hp,hq = r.fk_pose(); R=quat_R(*hq); print("hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), "handX", R[:,0].round(3), "yaw_deg", round(np.degrees(np.arctan2(R[1,0],R[0,0])),1))
EOF
timeout 120 python3 -u check.py

# openrua op 21
cd /workspace; timeout 600 python3 -u step2.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 22
cd /workspace; cat > step3.py <<'EOF'
import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step3")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
for z in [1.10, 1.045]:
    for attempt in range(3):
        tcp = r.move_tcp([-0.092, 0.023, z], q, 2.5)
        hp,hq = r.fk_pose()
        if abs(r.tcp_from_hand(hp,hq)[2]-z) < 0.005: break
js = r.gripper(0.0)
print("finger gap", js['panda_finger_joint1'] - js['panda_finger_joint2'])
EOF
timeout 900 python3 -u step3.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih3.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent3.png

# openrua op 23
cd /workspace; cat > step4.py <<'EOF'
import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step4")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
for attempt in range(3):
    r.move_tcp([-0.092, 0.023, 1.30], q, 3.0)
    hp,hq = r.fk_pose()
    if abs(r.tcp_from_hand(hp,hq)[2]-1.30) < 0.005: break
js = r.joints(); print("finger gap", js['panda_finger_joint1'] - js['panda_finger_joint2'])
EOF
timeout 900 python3 -u step4.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent4.png && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/bird_depth4.png

# openrua op 24
cd /workspace; cat > bookpose.py <<'EOF'
import numpy as np, sys
npy = sys.argv[1] if len(sys.argv)>1 else 'snaps/bird_depth4.npy'
d=np.load(npy)
fx=fy=579.4112549695428; cx=320; cy=240
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=(vv-cy)*d/fy - 0.2; Y=(uu-cx)*d/fx; Z=3.0-d
# book candidates: Z in 0.95..1.2, X in -0.3..0.2, Y in -0.15..0.25 (exclude caddy, robot)
m=(Z>0.95)&(Z<1.2)&(X>-0.3)&(X<0.2)&(Y>-0.15)&(Y<0.25)
# remove mug region (mug near -0.2, 0.14) -> mug Z max ~1.04
print('pts',m.sum())
for lo in [0.95,1.0,1.03,1.05,1.06,1.07]:
    mm=m&(Z>lo)
    if mm.sum()==0: continue
    pts=np.stack([X[mm],Y[mm]],1); c=pts-pts.mean(0); u,s,vt=np.linalg.svd(c,full_matrices=False)
    print(f'Z>{lo}: n={mm.sum()} center={pts.mean(0).round(4)} axis={vt[0].round(3)} along=({(c@vt[0]).min():.3f},{(c@vt[0]).max():.3f}) across=({(c@vt[1]).min():.3f},{(c@vt[1]).max():.3f}) Zmax={Z[mm].max():.3f}')
# check mug separately
mm=(Z>0.95)&(X>-0.3)&(X<-0.1)&(Y>0.08)&(Y<0.25)
print('mug-ish region n',mm.sum(), 'Zmax', Z[mm].max() if mm.sum() else None, 'center', (X[mm].mean(), Y[mm].mean()) if mm.sum() else None)
EOF
python3 bookpose.py snaps/bird_depth4.npy; echo ---- initial; python3 bookpose.py snaps/bird_depth.npy

# openrua op 25
cd /workspace; cat > park.py <<'EOF'
import math, numpy as np, sys
from robot import Robot, yaw_down_quat
r = Robot("park")
x,y,z = map(float, sys.argv[1:4]); yaw = float(sys.argv[4]) if len(sys.argv)>4 else 0.0
q = yaw_down_quat(math.radians(yaw - 45.0))
for attempt in range(3):
    tcp = r.move_tcp([x,y,z], q, 3.0)
    if tcp is not None and np.linalg.norm(tcp-[x,y,z]) < 0.005: break
EOF
timeout 600 python3 -u park.py -0.30 0.30 1.40 && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/bird_depth5.png && timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird5.png && python3 bookpose.py snaps/bird_depth5.npy

# openrua op 26
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load('snaps/bird_depth5.npy'); print('table Z', 3.0-d[400,320], 3.0-d[300,500], 'caddy back wall', (3.0-d[162:165,240:300]).max(), 'caddy floor', (3.0-d[170:180,265:295]).mean())
EOF

# openrua op 27
cd /workspace; cat > grasp.py <<'EOF'
import math, numpy as np, sys
from robot import Robot, yaw_down_quat, quat_R
bx, by, ztop, yaw = map(float, sys.argv[1:5])
r = Robot("grasp")
q = yaw_down_quat(math.radians(yaw - 45.0))
r.gripper(0.04)
def goto(z, secs=3.0):
    for attempt in range(3):
        tcp = r.move_tcp([bx, by, z], q, secs)
        if tcp is not None and np.linalg.norm(tcp-[bx,by,z]) < 0.005: return True
    return False
goto(ztop + 0.12)
goto(ztop + 0.05, 2.0)
goto(ztop - 0.03, 2.0)
js = r.gripper(0.0)
print("gap after close", js['panda_finger_joint1'] - js['panda_finger_joint2'])
goto(ztop + 0.03, 2.0)
js = r.joints(); print("gap after small lift", js['panda_finger_joint1'] - js['panda_finger_joint2'])
goto(ztop + 0.25, 3.0)
js = r.joints(); print("gap after lift", js['panda_finger_joint1'] - js['panda_finger_joint2'])
EOF
timeout 1200 python3 -u grasp.py -0.0905 0.0219 1.021 -40.8 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent6.png

# openrua op 28
cd /workspace; cat > place.py <<'EOF'
import math, numpy as np, sys
from robot import Robot, yaw_down_quat, quat_R
px, py, yaw = map(float, sys.argv[1:4])
r = Robot("place")
q = yaw_down_quat(math.radians(yaw - 45.0))
def goto(x,y,z, secs=3.0):
    for attempt in range(3):
        tcp = r.move_tcp([x, y, z], q, secs)
        if tcp is not None and np.linalg.norm(tcp-[x,y,z]) < 0.005: return True
    return False
def gap():
    js = r.joints(); return js['panda_finger_joint1'] - js['panda_finger_joint2']
goto(px, py, 1.27, 4.0); print("gap", gap())
hp,hq = r.fk_pose(); R=quat_R(*hq); print("handX", R[:,0].round(3), "handY", R[:,1].round(3))
EOF
timeout 900 python3 -u place.py -0.428 -0.142 -90 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih7.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent7.png

# openrua op 29
cd /workspace; cat > release.py <<'EOF'
import math, numpy as np, sys
from robot import Robot, yaw_down_quat, quat_R
px, py, yaw, zrel = map(float, sys.argv[1:5])
r = Robot("release")
q = yaw_down_quat(math.radians(yaw - 45.0))
def goto(x,y,z, secs=3.0):
    for attempt in range(3):
        tcp = r.move_tcp([x, y, z], q, secs)
        if tcp is not None and np.linalg.norm(tcp-[x,y,z]) < 0.005: return True
    return False
def gap():
    js = r.joints(); return js['panda_finger_joint1'] - js['panda_finger_joint2']
goto(px, py, 1.15, 2.5); print("gap", gap())
goto(px, py, zrel, 2.5); print("gap", gap())
r.gripper(0.04)
goto(px, py, 1.30, 3.0)
EOF
timeout 900 python3 -u release.py -0.428 -0.142 -90 1.065 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent8.png && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front8.png

# openrua op 30
cd /workspace; timeout 600 python3 -u park.py -0.30 0.35 1.40 && timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird9.png && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw snaps/bird_depth9.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent9.png && python3 -c "
import cv2
im=cv2.imread('snaps/agent9.png'); cv2.imwrite('snaps/agent9_crop.png', cv2.resize(im[140:300,100:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/bird9.png'); cv2.imwrite('snaps/bird9_crop.png', cv2.resize(im[140:230,190:330],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 31
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load('snaps/bird_depth9.npy'); fx=579.4112549695428
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
X=(vv-240)*d/fx-0.2; Y=(uu-320)*d/fx; Z=3.0-d
# region of the middle column back cell
m=(X>-0.47)&(X<-0.39)&(Y>-0.22)&(Y<-0.06)&(Z>0.92)&(Z<1.2)
print('n',m.sum(),'Z max',Z[m].max().round(3))
mm=m&(Z>1.06)   # above wall tops = book top
print('book-top pts',mm.sum(),'X',X[mm].min().round(3),X[mm].max().round(3),'Y',Y[mm].min().round(3),Y[mm].max().round(3),'Z',Z[mm].mean().round(3))
# the book on the table anywhere else?
o=(Z>0.95)&(Z<1.1)&(X>-0.3)&(X<0.3)&(Y>-0.3)&(Y<0.3)
print('other objects pts (excluding caddy region)', o.sum())
EOF

# openrua op 32
cd /workspace; python3 - <<'EOF'
import numpy as np
d=np.load('snaps/bird_depth9.npy'); fx=579.4112549695428
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
X=(vv-240)*d/fx-0.2; Y=(uu-320)*d/fx; Z=3.0-d
# interior of the middle-back cell (inside the walls)
m=(X>-0.452)&(X<-0.405)&(Y>-0.205)&(Y<-0.08)
print('interior pts',m.sum(),'Z range',Z[m].min().round(3),Z[m].max().round(3))
h,e=np.histogram(Z[m],bins=np.arange(0.88,1.08,0.01))
for a,b in zip(e,h): print(round(a,2),b)
mm=m&(Z>1.0)
print('book pts X',X[mm].min().round(3),X[mm].max().round(3),'Y',Y[mm].min().round(3),Y[mm].max().round(3),'Zmean',Z[mm].mean().round(3))
# check the book isn't on the table elsewhere
o=(Z>0.95)&(Z<1.1)&(X>-0.3)&(X<0.3)&(Y>-0.3)&(Y<0.3)
print('other pts', o.sum(), 'mug?', (X[o].mean(),Y[o].mean()) if o.sum() else None)
EOF
