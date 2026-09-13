#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iE "ik|fk|plan|scene" && echo ---- && ros2 node list

# openrua op 4
mkdir -p img && for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/birdview_depth.png; ros2 topic echo /birdview/color/camera_info --once | head -30; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12

# openrua op 6
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
# table depth estimate: median
print('median', np.median(d[np.isfinite(d)]))
# print depth at some points: cup approx (328,272), caddy (260,180), robot base
for (u,v) in [(328,272),(230,170),(280,170),(230,190),(280,190),(370,230),(120,290),(320,400)]:
    print((u,v), d[v,u])
"


# openrua op 7
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(v-cy)*d/fy-0.2; Y=(u-cx)*d/fx; Z=3.0-d
table=np.median(Z[np.isfinite(Z)])
print('table z',table)
H=Z-table
mask=(H>0.01)&(H<0.5)
# connected components
import cv2
n,lab=cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1,n):
    m=lab==i
    if m.sum()<30: continue
    vs,us=np.where(m)
    print(f'comp {i}: px={m.sum()} u[{us.min()},{us.max()}] v[{vs.min()},{vs.max()}] X[{X[m].min():.3f},{X[m].max():.3f}] Y[{Y[m].min():.3f},{Y[m].max():.3f}] Zmax={Z[m].max():.3f} Hmax={H[m].max():.3f}')
"


# openrua op 8
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ----; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -8; echo ----; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 9
python3 -c "
import cv2, numpy as np
im=cv2.imread('img/birdview.png')
crop=im[140:300,190:400]
cv2.imwrite('img/bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
d=np.load('img/birdview_depth.npy')
Z=3.0-d; H=Z-0.8804
crop=H[140:300,190:400]
vis=(np.clip(crop/0.2,0,1)*255).astype(np.uint8)
cv2.imwrite('img/bird_hcrop.png', cv2.resize(vis,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"


# openrua op 10
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(v-cy)*d/fy-0.2; Y=(u-cx)*d/fx; Z=3.0-d; H=Z-0.8804
# caddy region u 210..330, v 150..205
wall=(H>0.05)&(H<0.25)
sub=wall[150:210,210:340]
# column profile (sum along rows) -> vertical walls
colp=sub.sum(0); rowp=sub.sum(1)
print('cols with wall (u):',[ (210+i,int(c)) for i,c in enumerate(colp) if c>15])
print('rows with wall (v):',[ (150+i,int(c)) for i,c in enumerate(rowp) if c>20])
# wall heights
m=wall.copy(); m[:, :210]=False; m[:, 300:]=False; m[:150]=False; m[210:]=False
print('wall H median', np.median(H[m]), 'max', H[m].max())
# the divider in middle compartment: row where wall in u 270..300
for vv in range(150,210):
    row=wall[vv,265:300]
    if row.sum()>10: print('divider/wall row v',vv, 'X=',X[vv,280])
for uu in range(210,310):
    col=wall[155:200,uu]
    if col.sum()>25: print('vert wall u',uu,'Y=',Y[175,uu])
"


# openrua op 11
python3 -c "
import cv2
im=cv2.imread('img/agentview.png'); cv2.imwrite('img/agent_crop.png', cv2.resize(im[130:290,100:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/frontview.png'); cv2.imwrite('img/front_crop.png', cv2.resize(im[230:340,160:420],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(v-cy)*d/fy-0.2; Y=(u-cx)*d/fx; Z=3.0-d; H=Z-0.8804
reg=(slice(250,300),slice(295,350))
for lo in [0.02,0.06,0.10,0.13,0.15]:
    m=np.zeros_like(H,bool); m[reg]=H[reg]>lo
    if m.sum()==0: continue
    print(f'H>{lo}: n={m.sum()} X[{X[m].min():.3f},{X[m].max():.3f}] Y[{Y[m].min():.3f},{Y[m].max():.3f}] cX={X[m].mean():.3f} cY={Y[m].mean():.3f}')
# rim: H>0.13 ring; fit circle via mean of ring points
m=np.zeros_like(H,bool); m[reg]=H[reg]>0.13
xs,ys=X[m],Y[m]
# least squares circle fit
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle center',c[0],c[1],'radius',r)
# inner bottom depth of cup
print('cup inside z', Z[272,328], 'cup top', Z[m].max())
"


# openrua op 13
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw img/frontview_depth.png; ros2 topic echo /frontview/color/camera_info --once | grep -A9 "^k:"; timeout 15 ros2 run tf2_ros tf2_echo world frontview_optical_frame 2>&1 | grep -A5 Matrix | head -6

# openrua op 14
python3 -c "
import numpy as np, cv2
d=np.load('img/frontview_depth.npy')
f=579.4112549695428; cx=320; cy=240
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
P=np.stack([(u-cx)*d/f,(v-cy)*d/f,d,np.ones_like(d)],-1)
W=P@T.T
X,Y,Z=W[...,0],W[...,1],W[...,2]
# cup region in frontview approx u 290..370, v 290..360
reg=(slice(280,370),slice(285,375))
m=np.zeros(d.shape,bool); m[reg]=(Z[reg]>0.9)&(X[reg]>-0.2)&(X[reg]<0.05)
print('cup pts',m.sum(),'X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max(),'Z',Z[m].min(),Z[m].max())
# table Z near cup
print('table z sample', Z[400,320])
# caddy: region behind cup
reg=(slice(240,330),slice(170,300))
m=np.zeros(d.shape,bool); m[reg]=(Z[reg]>0.9)&(Z[reg]<1.1)&(X[reg]<-0.3)
print('caddy pts',m.sum(),'X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max(),'Z',Z[m].min(),Z[m].max())
# wall top heights: histogram of Z for caddy
h,e=np.histogram(Z[m],bins=20); print(list(zip(np.round(e[:-1],3),h)))
"


# openrua op 15
python3 -c "
import cv2
im=cv2.imread('img/sideview.png'); cv2.imwrite('img/side_crop.png', cv2.resize(im[200:340,280:520],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 16
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 17
timeout 600 python3 tools/action/ik_move.py 0.662 0.072 0.218 1 0 0 0 4 --at tcp; ros2 topic echo /joint_states --once | grep -A8 position

# openrua op 18
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library / CLI for this Panda station.

CLI:
  python3 rob.py state                      # joints, TCP pose (world), fingers, wrench
  python3 rob.py tcp X Y Z [yaw_deg] [secs] # move TCP to world pose, hand pointing down
  python3 rob.py joints p1,...,p7 [secs]
  python3 rob.py grip WIDTH
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # from tf world->panda_link0


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self._wr = {}
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self._js.clear()
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def wrench(self):
        self._wr.clear()
        end = 50
        while "m" not in self._wr and end > 0:
            self.spin(0.2); end -= 1
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def arm_seed(self, js=None):
        js = js or self.joints()
        s = JointState()
        for j in ARM:
            s.name.append(j); s.position.append(js[j])
        return s

    def fk_hand(self, js=None):
        """hand frame pose in BASE frame -> (pos, quat xyzw)"""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_seed(js)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, js=None):
        p, q = self.fk_hand(js)
        R = Rot.from_quat(q).as_matrix()
        tcp = p + R[:, 2] * TCP_OFF
        return tcp + BASE_IN_WORLD, q

    def ik_tcp_world(self, xyz, yaw_deg=0.0, seed=None):
        """IK for TCP at world xyz, hand pointing straight down, fingers
        opening along world Y rotated by yaw_deg about world Z."""
        R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        Rm = R.as_matrix()
        hand = np.asarray(xyz, float) - BASE_IN_WORLD - Rm[:, 2] * TCP_OFF
        q = R.as_quat()
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = self.arm_seed(seed)
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"IK failed code={None if r is None else r.error_code.val}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, secs=3.0, via=None):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for i, (pos, t) in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in pos])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, yaw_deg=0.0, secs=3.0):
        sol = self.ik_tcp_world(xyz, yaw_deg)
        code, err = self.move_joints(sol, secs)
        tcp, q = self.tcp_world()
        print(f"TCP world now: {np.round(tcp, 4)}  target {np.round(xyz, 4)}  quat {np.round(q, 3)}")
        return tcp

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers="
              f"{js['panda_finger_joint1']:.4f},{js['panda_finger_joint2']:.4f}")
        return js

    def state(self):
        js = self.joints()
        tcp, q = self.tcp_world(js)
        print("arm:", [round(js[j], 4) for j in ARM])
        print("fingers:", round(js["panda_finger_joint1"], 4), round(js["panda_finger_joint2"], 4))
        print("TCP world:", np.round(tcp, 4), "quat", np.round(q, 3))
        w = self.wrench()
        print("wrench:", None if w is None else np.round(w, 2))


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    a = sys.argv[2:]
    if cmd == "state":
        r.state()
    elif cmd == "tcp":
        xyz = [float(v) for v in a[:3]]
        yaw = float(a[3]) if len(a) > 3 else 0.0
        secs = float(a[4]) if len(a) > 4 else 3.0
        r.move_tcp(xyz, yaw, secs)
    elif cmd == "joints":
        pos = [float(v) for v in a[0].split(",")]
        r.move_joints(pos, float(a[1]) if len(a) > 1 else 3.0)
    elif cmd == "grip":
        r.gripper(float(a[0]))
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 19
timeout 120 python3 rob.py state

# openrua op 20
timeout 120 python3 -c "
import rob, rclpy
r=rob.Robot()
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link8','panda_link0']
req.robot_state.joint_state=r.arm_seed()
f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
res=f.result(); print(res.error_code, res.fk_link_names)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose)
"

# openrua op 21
timeout 120 python3 -c "
import rob, rclpy
r=rob.Robot()
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
req.header.frame_id='panda_link0'
req.robot_state.joint_state=r.arm_seed()
print(req.robot_state.joint_state)
f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=60)
res=f.result(); print(res.error_code, res.fk_link_names)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose)
"

# openrua op 22
timeout 200 python3 -c "
import rob, rclpy, time
r=rob.Robot()
from moveit_msgs.srv import GetPositionFK
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
req.robot_state.joint_state=r.arm_seed()
t=time.time()
f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=150)
res=f.result(); print(time.time()-t, res.error_code if res else None)
if res:
  for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose)
"

# openrua op 23
ros2 node list; ls -t .ros/log | head -3; tail -5 .ros/log/$(ls -t .ros/log | head -1)

# openrua op 24
timeout 100 python3 -c "
import rob, rclpy, time
r=rob.Robot()
t=time.time()
try:
    sol=r.ik_tcp_world([-0.088,0.072,1.13]); print('IK ok',time.time()-t, [round(x,3) for x in sol])
except Exception as e: print('ERR',e,time.time()-t)
"

# openrua op 25
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('img/robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -B2 -A6 -E '<joint name="(world|panda_hand_joint|panda_joint8|panda_hand_tcp)' img/robot.urdf | head -60; grep -n 'link name' img/robot.urdf

# openrua op 26
timeout 200 python3 -c "
import rob, rclpy, time, numpy as np
from moveit_msgs.srv import GetPositionIK
r=rob.Robot()
def ik(pos,q):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'
    req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    req.ik_request.robot_state.joint_state=r.arm_seed()
    req.ik_request.timeout.sec=2
    t=time.time(); f=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=60)
    res=f.result()
    if res is None: print('timeout'); return
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(pos,'code',res.error_code.val, round(time.time()-t,1), [round(sol.get(j,9),3) for j in rob.ARM] if res.error_code.val==1 else '')
q=[1,0,-0.028,0]
ik([0.457,0.0,0.358],q)      # base-frame current hand
ik([-0.293,0.0,1.27],q)      # world-frame current hand
"

# openrua op 27
timeout 200 python3 -c "
import rob, rclpy, time, numpy as np
from moveit_msgs.srv import GetPositionIK
r=rob.Robot()
def ik(pos,q):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'
    req.ik_request.pose_stamped.header.frame_id=''
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=[float(v) for v in pos]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]
    req.ik_request.robot_state.joint_state=r.arm_seed()
    req.ik_request.timeout.sec=2
    t=time.time(); f=r.ik.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=60)
    res=f.result()
    if res is None: print('timeout'); return
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(pos,'code',res.error_code.val, round(time.time()-t,1), [round(sol.get(j,9),3) for j in rob.ARM] if res.error_code.val==1 else '')
q=[1,0,-0.028,0]
ik([0.457,0.0,0.358],q)      # base-frame current hand
ik([-0.293,0.0,1.27],q)      # world-frame current hand
"

# openrua op 28
timeout 200 python3 -c "
import rob, rclpy, time, numpy as np
r=rob.Robot()
t=time.time()
js=r.joints()
sol=dict(zip(rob.ARM,[0.058, -0.162, -0.058, -2.445, -0.012, 2.227, 0.009]))
try:
    print('FK current', r.fk_hand(js), time.time()-t)
    print('FK iksol', r.fk_hand(sol), time.time()-t)
except Exception as e: print('ERR',e, time.time()-t)
"

# openrua op 29
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.75, 0.0, 0.912])  # from tf world->panda_link0',
 'BASE_IN_WORLD = np.zeros(3)  # MoveIt FK/IK here already answer in the WORLD frame (verified empirically)')
s=s.replace('req.ik_request.group_name = M["planning"]["group"]\n',
 'req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"\n')
open('rob.py','w').write(s)
EOF
grep -n "ik_link_name\|BASE_IN_WORLD =" rob.py

# openrua op 30
timeout 200 python3 -c "
import rob, numpy as np
r=rob.Robot()
for tgt in [[-0.088,0.072,1.13],[-0.088,0.072,1.015],[-0.44,-0.093,1.2],[-0.44,-0.093,1.13]]:
    try:
        sol=r.ik_tcp_world(tgt)
        tcp,q=r.tcp_world(dict(zip(rob.ARM,sol)))
        print(tgt,'->',[round(x,3) for x in sol],'FK tcp',np.round(tcp,4),'q',np.round(q,3))
    except Exception as e: print(tgt,'ERR',e)
"

# openrua op 31
timeout 900 python3 rob.py tcp -0.088 0.072 1.13 0 4

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent1.png

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd1.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview img/bird1.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('img/bd1.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
# fingertips: points with Z between 1.0 and 1.06 near cup
m=(Z>1.0)&(Z<1.07)&(X>-0.25)&(X<0.05)&(Y>-0.1)&(Y<0.2)
print('finger-ish pts',m.sum(),'X',X[m].min(),X[m].max(),'Y',Y[m].min(),Y[m].max())
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<5: continue
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),Z[mm].min(),Z[mm].max()))
im=cv2.imread('img/bird1.png'); cv2.imwrite('img/bird1_crop.png', cv2.resize(im[230:310,280:380],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 34
python3 -c "
import numpy as np, cv2
d=np.load('img/bd1.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
m=(Z>0.95)&(X>-0.3)&(X<0.1)&(Y>-0.15)&(Y<0.25)
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<5: continue
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),Z[mm].min(),Z[mm].max()))
im=cv2.imread('img/bird1.png'); cv2.imwrite('img/bird1_crop.png', cv2.resize(im[220:320,270:400],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 35
python3 -c "
import cv2
im=cv2.imread('img/agent1.png'); cv2.imwrite('img/agent1_crop.png', cv2.resize(im[150:400,200:460],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 36
timeout 60 python3 tools/perception/cam_snap.py sideview img/side1.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('img/side1.png'); cv2.imwrite('img/side1_crop.png', cv2.resize(im[100:330,200:450],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 37
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
print('wrench before', np.round(r.wrench(),2))
r.move_tcp([-0.088,0.072,1.045],0,2.5)
print('wrench mid', np.round(r.wrench(),2))
r.move_tcp([-0.088,0.072,1.012],0,2.0)
print('wrench after', np.round(r.wrench(),2))
"

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py sideview img/side2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih2.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('img/side2.png'); cv2.imwrite('img/side2_crop.png', cv2.resize(im[150:330,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 39
timeout 600 python3 rob.py grip 0.0

# openrua op 40
timeout 900 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.088,0.072,1.25],0,3)
print('wrench', np.round(r.wrench(),2))
js=r.joints(); print('fingers', round(js['panda_finger_joint1'],4), round(js['panda_finger_joint2'],4))
"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side3.png >/dev/null

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd2.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('img/bd2.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d; H=Z-0.8804
m=(H>0.13)&(H<0.2)&(X>-0.3)&(X<0.1)&(Y>-0.15)&(Y<0.25)
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<20: continue
    xs,ys=X[mm],Y[mm]
    A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; rad=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(xs.min(),xs.max(),ys.min(),ys.max(),Z[mm].min(),Z[mm].max()),'circle',np.round(c[:2],4),round(rad,4))
"

# openrua op 42
python3 -c "
import numpy as np, cv2
d=np.load('img/bd2.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d; H=Z-0.8804
m=(H>0.02)&(X>-0.3)&(X<0.15)&(Y>-0.2)&(Y<0.3)
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<20: continue
    xs,ys=X[mm],Y[mm]
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(xs.min(),xs.max(),ys.min(),ys.max(),Z[mm].min(),Z[mm].max()))
"

# openrua op 43
timeout 900 python3 rob.py tcp -0.30 0.25 1.30 0 3 && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd2.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('img/bd2.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d; H=Z-0.8804
m=(H>0.13)&(H<0.2)&(X>-0.3)&(X<0.15)&(Y>-0.2)&(Y<0.3)
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<20: continue
    xs,ys=X[mm],Y[mm]
    A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; rad=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(xs.min(),xs.max(),ys.min(),ys.max(),Z[mm].min(),Z[mm].max()),'circle',np.round(c[:2],4),round(rad,4))
"

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py birdview img/bird2.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('img/bd2.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d; H=Z-0.8804
m=(H>0.02)&(X>-0.3)&(X<0.3)&(Y>-0.2)&(Y<0.4)
n,lab=cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,n):
    mm=lab==i
    if mm.sum()<20: continue
    xs,ys=X[mm],Y[mm]
    print(i,mm.sum(),'X %.3f..%.3f Y %.3f..%.3f Z %.3f..%.3f'%(xs.min(),xs.max(),ys.min(),ys.max(),Z[mm].min(),Z[mm].max()))
im=cv2.imread('img/bird2.png'); cv2.imwrite('img/bird2_crop.png', cv2.resize(im[200:340,240:440],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 45
python3 -c "
import numpy as np, cv2
d=np.load('img/bd2.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d; H=Z-0.8804
reg=(X>-0.16)&(X<-0.02)&(Y>-0.08)&(Y<0.1)
for lo in [0.03,0.06,0.09,0.1]:
    m=reg&(H>lo)
    xs,ys=X[m],Y[m]
    print(f'H>{lo}: n={m.sum()} X[{xs.min():.3f},{xs.max():.3f}] Y[{ys.min():.3f},{ys.max():.3f}] Zmax={Z[m].max():.3f}')
m=reg&(H>0.09)&(Y>-0.03)
xs,ys=X[m],Y[m]
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; rad=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle',np.round(c[:2],4),round(rad,4))
# interior floor depth
cu=int(c[1]*f/2.0+320); cv=int((c[0]+0.2)*f/2.0+240)
print('center px',cu,cv,'Z there',Z[cv,cu])
"

# openrua op 46
timeout 1200 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.gripper(0.04)
r.move_tcp([-0.088,0.071,1.08],0,4)
r.move_tcp([-0.088,0.071,0.95],0,3)
print('wrench', np.round(r.wrench(),2))
js=r.gripper(0.0)
r.move_tcp([-0.088,0.071,1.0],0,2)
js=r.joints(); print('fingers after 5cm lift', round(js['panda_finger_joint1'],4), round(js['panda_finger_joint2'],4))
"

# openrua op 47
timeout 60 python3 tools/perception/cam_snap.py sideview img/side4.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('img/side4.png'); cv2.imwrite('img/side4_crop.png', cv2.resize(im[150:340,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 48
ps aux | grep -v grep | grep -iE "python|mujoco|robosuite|libero|bridge" | head; echo ----; find / -iname "*caddy*" 2>/dev/null | head; find / -iname "*.bddl" 2>/dev/null | head

# openrua op 49
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
# column u for Y=-0.14 at depth ~2.0
for Yt in [-0.18,-0.14,-0.10]:
    uu=int(round(Yt*f/2.0+320))
    print('Y target',Yt,'u',uu)
    for vv in range(150,210):
        print(f'  v={vv} X={X[vv,uu]:.3f} Z={Z[vv,uu]:.3f}')
" | awk 'NR%1==0' | head -130

# openrua op 50
python3 -c "
import numpy as np
d=np.load('img/frontview_depth.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
P=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1); W=P@T.T
X,Y,Z=W[...,0],W[...,1],W[...,2]
m=(X>-0.16)&(X<-0.02)&(Y>-0.08)&(Y<0.1)&(Z>0.88)&(Z<1.05)
zs=Z[m]; ys=Y[m]; xs=X[m]
for z0 in np.arange(0.88,1.05,0.01):
    mm=(zs>=z0)&(zs<z0+0.01)
    if mm.sum()>3: print(f'z {z0:.2f}-{z0+0.01:.2f}: n={mm.sum()} Y[{ys[mm].min():.3f},{ys[mm].max():.3f}] width={ys[mm].max()-ys[mm].min():.3f} Xmin={xs[mm].min():.3f}')
"

# openrua op 51
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.088,0.071,1.22],0,3)
r.move_tcp([-0.44,-0.096,1.22],0,5)
js=r.joints(); print('fingers', round(js['panda_finger_joint1'],4)); print('wrench', np.round(r.wrench(),2))
"; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent3.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side5.png >/dev/null

# openrua op 52
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
for i in range(3):
    tcp=r.move_tcp([-0.44,-0.096,1.22],0,5)
    if np.linalg.norm(tcp-np.array([-0.44,-0.096,1.22]))<0.005: break
js=r.joints(); print('fingers', round(js['panda_finger_joint1'],4)); print('wrench', np.round(r.wrench(),2))
"

# openrua op 53
timeout 60 python3 tools/perception/cam_snap.py agentview img/agent4.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side6.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih4.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('img/agent4.png'); cv2.imwrite('img/agent4_crop.png', cv2.resize(im[0:300,100:420],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 54
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
w0=r.wrench(); print('w0',np.round(w0,2))
for z in [1.13,1.10,1.08,1.065]:
    tcp=r.move_tcp([-0.442,-0.096,z],0,2.5)
    js=r.joints(); w=r.wrench()
    print(f'z={z} fingers={js[\"panda_finger_joint1\"]:.4f} wrench={np.round(w,2)} dW={np.round(w-w0,2)}')
"

# openrua op 55
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.436,-0.090,1.15],0,2.5)
w0=r.wrench(); print('w0',np.round(w0,2))
for z in [1.10,1.08,1.065,1.055,1.045]:
    tcp=r.move_tcp([-0.436,-0.090,z],0,2.0)
    js=r.joints(); w=r.wrench()
    print(f'z={z} fingers={js[\"panda_finger_joint1\"]:.4f} dW={np.round(w-w0,2)}')
    if abs(w[2]-w0[2])>8 or abs(tcp[2]-z)>0.01:
        print('contact -> stop'); break
"

# openrua op 56
timeout 60 python3 tools/perception/cam_snap.py sideview img/side7.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent7.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('img/side7.png'); cv2.imwrite('img/side7_crop.png', cv2.resize(im[80:330,330:640],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/agent7.png'); cv2.imwrite('img/agent7_crop.png', cv2.resize(im[0:300,100:420],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 57
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.436,-0.090,1.16],0,2.5)
js=r.joints(); print('fingers', round(js['panda_finger_joint1'],4), 'wrench', np.round(r.wrench(),2))
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd3.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview img/bird3.png >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('img/bd3.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
reg=(X>-0.55)&(X<-0.30)&(Y>-0.40)&(Y<0.0)
for lo,hi in [(1.17,1.23),(1.05,1.17),(0.95,1.0)]:
    m=reg&(Z>lo)&(Z<hi)
    if m.sum()==0: print(lo,hi,'none'); continue
    print(f'Z in [{lo},{hi}]: n={m.sum()} X[{X[m].min():.3f},{X[m].max():.3f}] Y[{Y[m].min():.3f},{Y[m].max():.3f}]')
# rim ring fit
m=reg&(Z>1.17)&(Z<1.23)
xs,ys=X[m],Y[m]
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; rad=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('rim circle',np.round(c[:2],4),round(rad,4))
im=cv2.imread('img/bird3.png'); cv2.imwrite('img/bird3_crop.png', cv2.resize(im[130:230,200:340],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 58
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw img/fd3.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/fd3.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
P=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1); W=P@T.T
X,Y,Z=W[...,0],W[...,1],W[...,2]
m=(X>-0.52)&(X<-0.33)&(Y>-0.22)&(Y<-0.02)&(Z>1.07)&(Z<1.21)
zs=Z[m]; ys=Y[m]; xs=X[m]
for z0 in np.arange(1.07,1.21,0.01):
    mm=(zs>=z0)&(zs<z0+0.01)
    if mm.sum()>3: print(f'z {z0:.2f}: n={mm.sum()} X[{xs[mm].min():.3f},{xs[mm].max():.3f}] Y[{ys[mm].min():.3f},{ys[mm].max():.3f}]  Xmax-front {xs[mm].max():.3f}')
"

# openrua op 59
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.452,-0.096,1.16],0,2.5)
w0=r.wrench()
for z in [1.13,1.115,1.10,1.09]:
    tcp=r.move_tcp([-0.452,-0.096,z],0,2.0)
    w=r.wrench(); js=r.joints()
    print(f'z={z} fingers={js[\"panda_finger_joint1\"]:.4f} dW={np.round(w-w0,2)}')
    if abs(w[2]-w0[2])>5 or abs(tcp[2]-z)>0.008:
        print('contact -> stop'); break
"

# openrua op 60
timeout 1500 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.gripper(0.04)
r.move_tcp([-0.452,-0.096,1.25],0,3)
" && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent8.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py sideview img/side8.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('img/agent8.png'); cv2.imwrite('img/agent8_crop.png', cv2.resize(im[100:300,100:420],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/side8.png'); cv2.imwrite('img/side8_crop.png', cv2.resize(im[150:340,330:560],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 61
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd4.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview img/bird4.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('img/bd4.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
reg=(X>-0.52)&(X<-0.30)&(Y>-0.25)&(Y<-0.06)
# cup = points above caddy wall tops in the middle section or in slots interior above floor
m=reg&(Z>1.0)&~((X<-0.470)|(Y<-0.205)|(Y>-0.08))
print('cup-ish pts',m.sum(),'X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# profile along X at Y=-0.14
uu=int(round(-0.14*f/2.0+320))
for vv in range(152,205,2): print(f'  v={vv} X={X[vv,uu]:.3f} Z={Z[vv,uu]:.3f}')
im=cv2.imread('img/bird4.png'); cv2.imwrite('img/bird4_crop.png', cv2.resize(im[140:220,200:320],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 62
python3 -c "
import numpy as np, cv2
d=np.load('img/bd4.npy'); f=579.4112549695428
im=cv2.imread('img/bird4.png')
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
yellow=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>100)
reg=(X>-0.5)&(X<-0.3)&(Y>-0.3)&(Y<-0.15)
m=yellow&reg
print('handle pts',m.sum(),'X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].min(),Z[m].max()))
# handle bar: the outermost -Y part
mb=m&(Y<Y[m].min()+0.012)
print('bar pts',mb.sum(),'X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[mb].min(),X[mb].max(),Y[mb].min(),Y[mb].max(),Z[mb].min(),Z[mb].max()))
# body cross-section widths along X (Y extent of cup points with Z>1.0 excluding handle)
body=reg&(Z>0.995)&~yellow&(Y>-0.205)&(Y<-0.08)
for x0 in np.arange(-0.43,-0.30,0.01):
    mm=body&(X>=x0)&(X<x0+0.01)
    if mm.sum()>3: print(f'X {x0:.2f}: Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] width={Y[mm].max()-Y[mm].min():.3f} Ztop={Z[mm].max():.3f}')
"

# openrua op 63
python3 -c "
import numpy as np
d=np.load('img/bd4.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
xs=np.arange(-0.49,-0.28,0.01); ys=np.arange(-0.30,-0.04,0.01)
print('Ztop grid (cm above 0.9). rows X, cols Y')
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for x0 in xs:
    row=[]
    for y0 in ys:
        m=(X>=x0)&(X<x0+0.01)&(Y>=y0)&(Y<y0+0.01)
        row.append(f'{(Z[m].max()-0.9)*100:4.0f}' if m.sum()>0 else '   .')
    print(f'{x0*100:5.0f} '+' '.join(row))
"

# openrua op 64
timeout 60 python3 tools/perception/cam_snap.py frontview img/front5.png && timeout 60 python3 tools/perception/cam_snap.py sideview img/side5.png && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent5.png

# openrua op 65
python3 -c "
import numpy as np
d=np.load('img/birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
xs=np.arange(-0.49,-0.30,0.01); ys=np.arange(-0.30,-0.04,0.01)
print('INITIAL scene, Ztop grid (cm above 0.9). rows X, cols Y')
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for x0 in xs:
    row=[]
    for y0 in ys:
        m=(X>=x0)&(X<x0+0.01)&(Y>=y0)&(Y<y0+0.01)
        row.append(f'{(Z[m].max()-0.9)*100:4.0f}' if m.sum()>0 else '   .')
    print(f'{x0*100:5.0f} '+' '.join(row))
"

# openrua op 66
python3 -c "
import numpy as np
d=np.load('img/fd3.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
pc=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1)
pw=pc@T.T
X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(d)
# the cup was hanging: TCP approx (-0.44,-0.096,1.12): cup center (-0.44,-0.143), bottom ~1.05, rim ~1.16
m=ok&(X>-0.55)&(X<-0.30)&(Y>-0.30)&(Y<0.0)&(Z>1.03)&(Z<1.20)
print('pts',m.sum())
for z0 in np.arange(1.03,1.20,0.01):
    mm=m&(Z>=z0)&(Z<z0+0.01)
    if mm.sum()>2:
        ys=Y[mm]; xs=X[mm]
        print(f'z {z0:.2f}: n={mm.sum():4d} Y[{ys.min():.3f},{ys.max():.3f}] w={ys.max()-ys.min():.3f}  X[{xs.min():.3f},{xs.max():.3f}]')
"

# openrua op 67
ls -la img/ | head -60; python3 -c "
import rob; r=rob.Robot(); r.state()" 2>&1 | tail -5

# openrua op 68
python3 -c "
import numpy as np
d=np.load('img/frontview_depth.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
pc=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1)
pw=pc@T.T; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(d)
m=ok&(X>-0.25)&(X<0.05)&(Y>-0.15)&(Y<0.15)&(Z>0.90)&(Z<1.10)
print('pts',m.sum(), 'Zrange',Z[m].min(),Z[m].max())
for z0 in np.arange(0.90,1.08,0.01):
    mm=m&(Z>=z0)&(Z<z0+0.01)
    if mm.sum()>2:
        ys=Y[mm]; xs=X[mm]
        # body only: exclude handle side by taking +Y edge and the X-frontmost point
        print(f'z {z0:.2f}: n={mm.sum():4d} Y[{ys.min():.3f},{ys.max():.3f}] Xfront={xs.max():.3f} Xback={xs.min():.3f}')
"

# openrua op 69
python3 -c "
import numpy as np
d=np.load('img/frontview_depth.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
pc=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1)
pw=pc@T.T; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(d)
m=ok&(X>-0.16)&(X<0.0)&(Y>-0.12)&(Y<0.12)&(Z>0.92)&(Z<1.06)
for z0 in np.arange(0.93,1.05,0.005):
    mm=m&(Z>=z0)&(Z<z0+0.005)
    if mm.sum()>2:
        ys=Y[mm]; xs=X[mm]
        # +Y edge is body (handle at -Y). center Y=0.0237
        print(f'z {z0:.3f}: n={mm.sum():4d} Y[{ys.min():.3f},{ys.max():.3f}] r+={ys.max()-0.0237:.3f} Xfront={xs.max():.3f} (rX={xs.max()+0.0883:.3f})')
"

# openrua op 70
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def ik_tcp_world(self, xyz, yaw_deg=0.0, seed=None):\\n        \\"\\"\\"IK for TCP at world xyz, hand pointing straight down, fingers\\n        opening along world Y rotated by yaw_deg about world Z.\\"\\"\\"\\n        R = Rot.from_euler(\\"z\\", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])\\n        Rm = R.as_matrix()", "new_string": "    @staticmethod\\n    def hand_R(point, pads):\\n        \\"\\"\\"Rotation whose columns are hand X/Y/Z in world: Z = pointing\\n        direction (approach), Y = finger opening axis.\\"\\"\\"\\n        z = np.asarray(point, float); z /= np.linalg.norm(z)\\n        y = np.asarray(pads, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)\\n        x = np.cross(y, z)\\n        return np.stack([x, y, z], axis=1)\\n\\n    def ik_tcp_world(self, xyz, yaw_deg=0.0, seed=None, R=None):\\n        \\"\\"\\"IK for TCP at world xyz. Default: hand pointing straight down,\\n        fingers opening along world Y rotated by yaw_deg about world Z.\\n        R (3x3, see hand_R) overrides the orientation.\\"\\"\\"\\n        if R is None:\\n            R = Rot.from_euler(\\"z\\", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])\\n        else:\\n            R = Rot.from_matrix(np.asarray(R, float))\\n        Rm = R.as_matrix()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 71
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_tcp(self, xyz, yaw_deg=0.0, secs=3.0):\\n        sol = self.ik_tcp_world(xyz, yaw_deg)\\n        code, err = self.move_joints(sol, secs)\\n        tcp, q = self.tcp_world()\\n        print(f\\"TCP world now: {np.round(tcp, 4)}  target {np.round(xyz, 4)}  quat {np.round(q, 3)}\\")\\n        return tcp", "new_string": "    def move_tcp(self, xyz, yaw_deg=0.0, secs=3.0, R=None):\\n        sol = self.ik_tcp_world(xyz, yaw_deg, R=R)\\n        code, err = self.move_joints(sol, secs)\\n        if code == -5 and err > 0.02:  # controller lagged on a long move: resend\\n            code, err = self.move_joints(sol, secs)\\n        tcp, q = self.tcp_world()\\n        print(f\\"TCP world now: {np.round(tcp, 4)}  target {np.round(xyz, 4)}  quat {np.round(q, 3)}\\")\\n        return tcp\\n\\n    def hand_axes(self):\\n        \\"\\"\\"current hand axes in world: (pointing, pads) unit vectors\\"\\"\\"\\n        _, q = self.fk_hand()\\n        Rm = Rot.from_quat(q).as_matrix()\\n        return Rm[:, 2], Rm[:, 1]", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 72
timeout 120 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
print(R)
for p in [(-0.2,-0.175,1.25),(-0.22,-0.175,1.045),(-0.325,-0.175,1.045),(-0.325,-0.175,1.20)]:
    try:
        s=r.ik_tcp_world(p,R=R); print(p,'OK',np.round(s,3))
    except Exception as e: print(p,e)
" 2>&1 | grep -v '^\[' 

# openrua op 73
timeout 120 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
for p in [(-0.29,-0.175,1.045),(-0.29,-0.175,1.20),(-0.27,-0.175,1.045),(-0.29,-0.175,1.30),(-0.36,-0.175,1.30)]:
    try:
        s=r.ik_tcp_world(p,R=R); print(p,'OK',np.round(s,3))
    except Exception as e: print(p,e)
" 2>&1 | grep -v '^\['

# openrua op 74
python3 -c "
import numpy as np
d=np.load('img/bd4.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
m=(Y>-0.18)&(Y<-0.17)&(X>-0.34)&(X<-0.28)
xs=X[m]; zs=Z[m]
o=np.argsort(xs)
for x,z in zip(xs[o],zs[o]): print(f'{x:.4f} {z:.4f}')
" | awk 'NR%2==1'

# openrua op 75
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
r.move_tcp([-0.40,-0.175,1.40],0,4)
print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.36,-0.175,1.30],secs=6,R=R)
print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.27,-0.175,1.20],secs=4,R=R)
print('wrench',np.round(r.wrench(),1))
print('axes',[np.round(a,3) for a in r.hand_axes()])
" 2>&1 | grep -v '^\['

# openrua op 76
timeout 60 python3 tools/perception/cam_snap.py sideview img/side8.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih8.png >/dev/null; echo ok

# openrua op 77
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
r.move_tcp([-0.27,-0.175,1.04],secs=4,R=R)
print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.32,-0.175,1.04],secs=3,R=R)
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front9.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side9.png >/dev/null

# openrua op 78
timeout 120 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.gripper(0.0)
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['

# openrua op 79
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih9.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent9.png >/dev/null; echo ok

# openrua op 80
timeout 200 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
r.move_tcp([-0.32,-0.175,1.08],secs=3,R=R)
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4))
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py sideview img/side10.png >/dev/null

# openrua op 81
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd5.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/bd5.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
xs=np.arange(-0.49,-0.26,0.01); ys=np.arange(-0.30,-0.04,0.01)
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for x0 in xs:
    row=[]
    for y0 in ys:
        m=(X>=x0)&(X<x0+0.01)&(Y>=y0)&(Y<y0+0.01)
        row.append(f'{(Z[m].max()-0.9)*100:4.0f}' if m.sum()>0 else '   .')
    print(f'{x0*100:5.0f} '+' '.join(row))
"

# openrua op 82
timeout 200 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([-1,0,0],[0,0,1])
r.move_tcp([-0.32,-0.175,1.22],secs=4,R=R)
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4))
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py sideview img/side11.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview img/front11.png >/dev/null

# openrua op 83
timeout 200 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=[js[j] for j in rob.ARM]; print('cur',np.round(cur,3))
R2=rob.Robot.hand_R([-1,0,0],[0,1,0])
s=r.ik_tcp_world([-0.32,-0.175,1.22],R=R2); print('roll ',np.round(s,3), 'delta',np.round(np.array(s)-cur,2))
R3=rob.Robot.hand_R([0,1,0],[1,0,0])
s3=r.ik_tcp_world([-0.32,-0.175,1.22],R=R3,seed=dict(zip(rob.ARM,s))|{k:js[k] for k in js if k not in rob.ARM}); print('yaw  ',np.round(s3,3),'delta',np.round(np.array(s3)-s,2))
for p in [(-0.15,-0.075,1.15),(-0.15,-0.075,0.96),(-0.20,-0.075,0.96),(-0.25,-0.075,0.96)]:
    try: print(p,np.round(r.ik_tcp_world(p,R=R3,seed=dict(zip(rob.ARM,s3))),3))
    except Exception as e: print(p,e)
" 2>&1 | grep -v '^\['

# openrua op 84
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
P=[-0.32,-0.175,1.22]
def ik(point,pads,seed):
    return np.array(r.ik_tcp_world(P,R=rob.Robot.hand_R(point,pads),seed=dict(zip(rob.ARM,seed))))
# option A: roll -90 (pads +Y) then yaw to -Y pointing
sA1=ik([-1,0,0],[0,1,0],cur); print('A roll',np.round(sA1,2),'d',np.round(np.abs(sA1-cur).max(),2))
for pads in ([-1,0,0],[1,0,0]):
    try:
        sA2=ik([0,-1,0],pads,sA1); print('A yaw->-Y pads',pads,np.round(sA2,2),'d',np.round(np.abs(sA2-sA1).max(),2))
    except Exception as e: print('A yaw',pads,e)
# option B: yaw first (pads stay +Z), pointing -Y or +Y
for pt in ([0,-1,0],[0,1,0]):
    try:
        sB1=ik(pt,[0,0,1],cur); print('B yaw',pt,np.round(sB1,2),'d',np.round(np.abs(sB1-cur).max(),2))
        for pads in ([1,0,0],[-1,0,0]):
            try:
                sB2=ik(pt,pads,sB1); print('   B roll pads',pads,np.round(sB2,2),'d',np.round(np.abs(sB2-sB1).max(),2))
            except Exception as e: print('   B roll',pads,e)
    except Exception as e: print('B yaw',pt,e)
" 2>&1 | grep -v '^\['

# openrua op 85
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
def ik(P,point,pads,seed):
    return np.array(r.ik_tcp_world(P,R=rob.Robot.hand_R(point,pads),seed=dict(zip(rob.ARM,seed))))
P=[-0.32,-0.175,1.22]
sA1=ik(P,[-1,0,0],[0,1,0],cur); print('A roll',np.round(sA1,2))
# intermediate yaw -45 deg: pointing (-1,1,0)/sqrt2, pads (1,1,0)/sqrt2
try:
    s45=ik(P,[-1,1,0],[1,1,0],sA1); print('yaw-45',np.round(s45,2),'d',np.round(np.abs(s45-sA1).max(),2))
    s90=ik(P,[0,1,0],[1,0,0],s45); print('yaw-90',np.round(s90,2),'d',np.round(np.abs(s90-s45).max(),2))
except Exception as e: print(e)
try:
    s90b=ik(P,[0,1,0],[1,0,0],sA1); print('yaw-90 direct',np.round(s90b,2),'d',np.round(np.abs(s90b-sA1).max(),2))
except Exception as e: print(e)
# also try at different positions (closer to robot / higher)
for P2 in ([-0.35,-0.10,1.25],[-0.30,-0.05,1.25],[-0.40,-0.15,1.30]):
    try:
        s=ik(P2,[0,1,0],[1,0,0],sA1); print(P2,np.round(s,2),'d',np.round(np.abs(s-sA1).max(),2))
    except Exception as e: print(P2,e)
" 2>&1 | grep -v '^\['

# openrua op 86
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
def ik(P,point,pads,seed):
    return np.array(r.ik_tcp_world(P,R=rob.Robot.hand_R(point,pads),seed=dict(zip(rob.ARM,seed))))
P=[-0.32,-0.175,1.22]
# flipped grasp (pads -Z)
s0=ik(P,[-1,0,0],[0,0,-1],cur); print('flip grasp',np.round(s0,2))
s1=ik(P,[-1,0,0],[0,-1,0],s0); print('roll handle-up',np.round(s1,2),'d',np.round(np.abs(s1-s0).max(),2))
for pt,pads in (([0,1,0],[-1,0,0]),([0,-1,0],[1,0,0])):
    try:
        s2=ik(P,pt,pads,s1); print('yaw',pt,np.round(s2,2),'d',np.round(np.abs(s2-s1).max(),2))
        for P3 in ([-0.15,-0.075,0.96],[-0.20,-0.02,0.96],[-0.20,0.03,0.96]):
            try:
                s3=ik(P3,pt,pads,s2); print('    lay',P3,np.round(s3,2),'d',np.round(np.abs(s3-s2).max(),2))
            except Exception as e: print('    lay',P3,e)
    except Exception as e: print('yaw',pt,e)
" 2>&1 | grep -v '^\['

# openrua op 87
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
def ik(P,point,pads,seed):
    return np.array(r.ik_tcp_world(P,R=rob.Robot.hand_R(point,pads),seed=dict(zip(rob.ARM,seed))))
P=[-0.32,-0.175,1.30]
s0=ik(P,[-1,0,0],[0,0,1],cur); print('raise',np.round(s0,2),'d',np.round(np.abs(s0-cur).max(),2))
s45=ik(P,[-1,0,-1],[-1,0,1],s0); print('pitch45',np.round(s45,2),'d',np.round(np.abs(s45-s0).max(),2))
s90=ik(P,[0,0,-1],[-1,0,0],s45); print('pitch90',np.round(s90,2),'d',np.round(np.abs(s90-s45).max(),2))
for P3 in ([-0.25,-0.02,1.20],[-0.25,-0.02,0.995]):
    s3=ik(P3,[0,0,-1],[-1,0,0],s90); print('place',P3,np.round(s3,2),'d',np.round(np.abs(s3-s90).max(),2)); s90=s3
" 2>&1 | grep -v '^\['

# openrua op 88
timeout 400 python3 -c "
import rob, numpy as np
r=rob.Robot()
P=[-0.32,-0.175,1.30]
r.move_tcp(P,secs=3,R=rob.Robot.hand_R([-1,0,0],[0,0,1]))
r.move_tcp(P,secs=4,R=rob.Robot.hand_R([-1,0,-1],[-1,0,1]))
r.move_tcp(P,secs=4,R=rob.Robot.hand_R([0,0,-1],[-1,0,0]))
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4))
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py sideview img/side12.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview img/front12.png >/dev/null

# openrua op 89
timeout 400 python3 -c "
import rob, numpy as np
r=rob.Robot()
R=rob.Robot.hand_R([0,0,-1],[-1,0,0])
r.move_tcp([-0.25,-0.02,1.20],secs=4,R=R)
r.move_tcp([-0.25,-0.02,1.03],secs=3,R=R)
print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.25,-0.02,0.998],secs=2,R=R)
print('wrench',np.round(r.wrench(),1))
r.gripper(0.04)
r.move_tcp([-0.25,-0.02,1.15],secs=3,R=R)
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd6.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview img/bird6.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview img/front13.png >/dev/null

# openrua op 90
python3 -c "
import numpy as np
d=np.load('img/bd6.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
reg=(X>-0.32)&(X<-0.10)&(Y>-0.16)&(Y<0.10)&(Z>0.905)
print('pts',reg.sum(),'Zmax',Z[reg].max())
rim=reg&(Z>0.975)
print('rim: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[rim].min(),X[rim].max(),Y[rim].min(),Y[rim].max(),Z[rim].min(),Z[rim].max()))
cx=(X[rim].min()+X[rim].max())/2; cy=(Y[rim].min()+Y[rim].max())/2
print('rim center approx',round(cx,4),round(cy,4),'diam X',round(X[rim].max()-X[rim].min(),3),'Y',round(Y[rim].max()-Y[rim].min(),3))
# handle: points at Y below the rim's min Y (i.e. -Y side) between 0.905 and 0.975
h=reg&(Z<0.975)&(Y<Y[rim].min()+0.005)
print('handle pts',h.sum())
if h.sum():
  print('handle: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max()))
  ys=np.arange(Y[h].min(),Y[h].max()+0.004,0.004)
  for y0 in ys:
    mm=h&(Y>=y0)&(Y<y0+0.004)
    if mm.sum(): print(f'  Y {y0:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Ztop={Z[mm].max():.3f} Zmin={Z[mm].min():.3f}')
"

# openrua op 91
python3 -c "
import numpy as np
d=np.load('img/bd6.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
reg=(X>-0.32)&(X<-0.10)&(Y>-0.16)&(Y<0.10)
rim=reg&(Z>0.975)&(Z<1.02)
print('rim: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f] n=%d'%(X[rim].min(),X[rim].max(),Y[rim].min(),Y[rim].max(),Z[rim].min(),Z[rim].max(),rim.sum()))
cx=(X[rim].min()+X[rim].max())/2; cy=(Y[rim].min()+Y[rim].max())/2
print('rim center',round(cx,4),round(cy,4),'diam',round(X[rim].max()-X[rim].min(),3),round(Y[rim].max()-Y[rim].min(),3))
# fit circle to rim points
A=np.c_[2*X[rim],2*Y[rim],np.ones(rim.sum())]; b=X[rim]**2+Y[rim]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print('circle fit center',round(c[0],4),round(c[1],4),'r',round(r,4))
h=reg&(Z>0.905)&(Z<0.975)&(Y<c[1]-0.055)
print('handle: X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f] n=%d'%(X[h].min(),X[h].max(),Y[h].min(),Y[h].max(),Z[h].min(),Z[h].max(),h.sum()))
for y0 in np.arange(Y[h].min(),Y[h].max()+0.003,0.003):
    mm=h&(Y>=y0)&(Y<y0+0.003)
    if mm.sum(): print(f'  Y {y0:.3f}: n={mm.sum():3d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Ztop={Z[mm].max():.3f}')
print('hand Z>1.05 region X[%.3f,%.3f] Y[%.3f,%.3f]'%tuple([X[reg&(Z>1.05)].min(),X[reg&(Z>1.05)].max(),Y[reg&(Z>1.05)].min(),Y[reg&(Z>1.05)].max()]))
"

# openrua op 92
timeout 300 python3 -c "
import rob, numpy as np
r=rob.Robot()
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
def ik(P,point,pads,seed):
    return np.array(r.ik_tcp_world(P,R=rob.Robot.hand_R(point,pads),seed=dict(zip(rob.ARM,seed))))
a=np.deg2rad(35); pt=[0,np.sin(a),-np.cos(a)]
for pads in ([1,0,0],[-1,0,0]):
    try:
        s1=ik([-0.2145,-0.094,1.10],pt,pads,cur); print('pads',pads,'pre',np.round(s1,2),'d',np.round(np.abs(s1-cur).max(),2))
        s2=ik([-0.2145,-0.094,0.943],pt,pads,s1); print('   grasp',np.round(s2,2),'d',np.round(np.abs(s2-s1).max(),2))
        s3=ik([-0.2145,-0.094,1.25],pt,pads,s2); print('   lift',np.round(s3,2),'d',np.round(np.abs(s3-s2).max(),2))
        s4=ik([-0.43,-0.147,1.25],pt,pads,s3); print('   over slot',np.round(s4,2),'d',np.round(np.abs(s4-s3).max(),2))
        s5=ik([-0.43,-0.147,1.11],pt,pads,s4); print('   lower',np.round(s5,2),'d',np.round(np.abs(s5-s4).max(),2))
        pt2=[0,-np.cos(a),-np.sin(a)]
        s6=ik([-0.43,-0.147,1.25],pt2,pads,s4); print('   alt rolled -Y',np.round(s6,2),'d',np.round(np.abs(s6-s4).max(),2))
    except Exception as e: print(pads,e)
" 2>&1 | grep -v '^\['

# openrua op 93
timeout 400 python3 -c "
import rob, numpy as np
r=rob.Robot()
a=np.deg2rad(35); R=rob.Robot.hand_R([0,np.sin(a),-np.cos(a)],[-1,0,0])
r.move_tcp([-0.2145,-0.094,1.10],secs=4,R=R)
print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.2145,-0.094,0.99],secs=3,R=R)
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front14.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih14.png >/dev/null

# openrua op 94
timeout 400 python3 -c "
import rob, numpy as np
r=rob.Robot()
a=np.deg2rad(35); R=rob.Robot.hand_R([0,np.sin(a),-np.cos(a)],[-1,0,0])
r.move_tcp([-0.2145,-0.094,0.945],secs=3,R=R)
print('wrench',np.round(r.wrench(),1))
r.gripper(0.0)
print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front15.png >/dev/null

# openrua op 95
timeout 400 python3 -c "
import rob, numpy as np
r=rob.Robot()
a=np.deg2rad(35); R=rob.Robot.hand_R([0,np.sin(a),-np.cos(a)],[-1,0,0])
r.move_tcp([-0.2145,-0.094,1.05],secs=4,R=R)
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4)); print('wrench',np.round(r.wrench(),1))
r.move_tcp([-0.2145,-0.094,1.25],secs=5,R=R)
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4)); print('wrench',np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front16.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side16.png >/dev/null

# openrua op 96
timeout 400 python3 -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r=rob.Robot()
a=np.deg2rad(35); pt=np.array([0,np.sin(a),-np.cos(a)]); pads=np.array([-1,0,0])
js=r.joints(); cur=np.array([js[j] for j in rob.ARM])
P=[-0.2145,-0.094,1.25]
sols=[]
seed=cur
for th in (-45,-90):
    Rx=Rot.from_euler('x',th,degrees=True).as_matrix()
    R=rob.Robot.hand_R(Rx@pt,Rx@pads)
    s=np.array(r.ik_tcp_world(P,R=R,seed=dict(zip(rob.ARM,seed)))); print(th,np.round(s,2),'d',np.round(np.abs(s-seed).max(),2)); seed=s
    sols.append((R,s))
for R,s in sols:
    r.move_joints(s,5)
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4)); print('wrench',np.round(r.wrench(),1))
print('axes',[np.round(a,3) for a in r.hand_axes()])
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front17.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side17.png >/dev/null

# openrua op 97
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw img/fd17.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/fd17.npy'); f=579.4112549695428
T=np.array([[0,0.258,-0.966,1.0],[1,0,0,0],[0,-0.966,-0.258,1.48],[0,0,0,1]])
v,u=np.mgrid[0:480,0:640]
pc=np.stack([(u-320)*d/f,(v-240)*d/f,d,np.ones_like(d)],-1)
pw=pc@T.T; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(d)
m=ok&(X>-0.40)&(X<-0.05)&(Y>-0.30)&(Y<0.10)&(Z>0.98)&(Z<1.30)
print('pts',m.sum())
for z0 in np.arange(0.98,1.30,0.02):
    mm=m&(Z>=z0)&(Z<z0+0.02)
    if mm.sum()>2: print(f'z {z0:.2f}: n={mm.sum():4d} X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 98
timeout 500 python3 -c "
import rob, numpy as np
r=rob.Robot()
pt=np.array([0,-0.818,-0.576]); R=rob.Robot.hand_R(pt,[-1,0,0])
r.move_tcp([-0.43,-0.1315,1.25],secs=5,R=R)
print('wrench',np.round(r.wrench(),1))
for z in (1.16,1.13,1.11,1.10,1.09):
    r.move_tcp([-0.43,-0.1315,z],secs=2,R=R)
    w=r.wrench(); print('z',z,'wrench',np.round(w,1))
    if abs(w[1]-4.4)>1.5 or abs(w[2]+3.1)>1.5 or abs(w[0])>1.5: print('CONTACT'); break
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py frontview img/front18.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview img/side18.png >/dev/null

# openrua op 99
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd7.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/bd7.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
xs=np.arange(-0.50,-0.36,0.01); ys=np.arange(-0.24,-0.04,0.01)
print('      '+' '.join(f'{y*100:4.0f}' for y in ys))
for x0 in xs:
    row=[]
    for y0 in ys:
        m=(X>=x0)&(X<x0+0.01)&(Y>=y0)&(Y<y0+0.01)
        row.append(f'{(Z[m].max()-0.9)*100:4.0f}' if m.sum()>0 else '   .')
    print(f'{x0*100:5.0f} '+' '.join(row))
cup=(X>-0.50)&(X<-0.36)&(Y>-0.22)&(Y<-0.06)&(Z>1.06)&(Z<1.20)
print('cup-ish pts X[%.3f,%.3f] Y[%.3f,%.3f] Z[%.3f,%.3f]'%(X[cup].min(),X[cup].max(),Y[cup].min(),Y[cup].max(),Z[cup].min(),Z[cup].max()))
"

# openrua op 100
timeout 500 python3 -c "
import rob, numpy as np
r=rob.Robot()
pt=np.array([0,-0.818,-0.576]); R=rob.Robot.hand_R(pt,[-1,0,0])
r.move_tcp([-0.415,-0.1315,1.20],secs=3,R=R)
w0=r.wrench(); print('free wrench',np.round(w0,1))
for z in (1.17,1.15,1.13,1.12,1.11,1.10,1.09,1.08):
    r.move_tcp([-0.415,-0.1315,z],secs=2,R=R)
    w=r.wrench(); print('z',z,'wrench',np.round(w,1))
    if np.abs(w[:3]-w0[:3]).max()>1.5: print('CONTACT'); break
js=r.joints(); print('fingers',round(js['panda_finger_joint1'],4),round(js['panda_finger_joint2'],4))
" 2>&1 | grep -v '^\['

# openrua op 101
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd8.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/bd8.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
for yc in (-0.19,-0.17,-0.15,-0.13):
    m=(Y>yc-0.005)&(Y<yc+0.005)&(X>-0.50)&(X<-0.36)
    xs=X[m]; zs=Z[m]; o=np.argsort(xs)
    print(f'Y={yc}: '+' '.join(f'{x:.3f}/{z:.3f}' for x,z in zip(xs[o][::2],zs[o][::2])))
"

# openrua op 102
timeout 120 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.gripper(0.04)
print('wrench after release', np.round(r.wrench(),1))
R=rob.Robot.hand_R([0,-0.818,-0.576],[-1,0,0])
r.move_tcp([-0.41,-0.135,1.20],secs=2,R=R)
r.move_tcp([-0.41,-0.16,1.32],secs=2,R=R)
print('wrench free', np.round(r.wrench(),1))
" 2>&1 | grep -v '^\['

# openrua op 103
for c in birdview agentview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c img/${c}_final.png >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd9.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/bd9.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
# height grid over caddy region
xs=np.arange(-0.49,-0.33,0.01); ys=np.arange(-0.24,-0.06,0.01)
print('      '+' '.join(f'{y:6.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(X>x-0.005)&(X<x+0.005)&(Y>y-0.005)&(Y<y+0.005)
        row.append(f'{np.median(Z[m]):6.3f}' if m.any() else '   nan')
    print(f'{x:6.2f} '+' '.join(row))
"

# openrua op 104
timeout 120 python3 -c "
import rob, numpy as np
r=rob.Robot()
r.move_tcp([-0.30,0.25,1.45],secs=3)
" 2>&1 | grep -v '^\['; for c in birdview agentview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c img/${c}_final.png >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd10.png >/dev/null; python3 -c "
import numpy as np
d=np.load('img/bd10.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
xs=np.arange(-0.49,-0.33,0.01); ys=np.arange(-0.24,-0.06,0.01)
print('      '+' '.join(f'{y:6.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(X>x-0.005)&(X<x+0.005)&(Y>y-0.005)&(Y<y+0.005)
        row.append(f'{np.median(Z[m]):6.3f}' if m.any() else '   nan')
    print(f'{x:6.2f} '+' '.join(row))
"

# openrua op 105
timeout 120 python3 -c "
import rob
r=rob.Robot(); r.gripper(0.04); r.move_tcp([-0.30,0.25,1.45],secs=2)
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw img/bd11.png >/dev/null; python3 -c "
import numpy as np
a=np.load('img/bd10.npy'); b=np.load('img/bd11.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*a/f-0.2; Y=(u-320)*a/f
m=(X>-0.48)&(X<-0.36)&(Y>-0.23)&(Y<-0.09)&np.isfinite(a)&np.isfinite(b)
print('max |dz| over cup region between two frames:', np.abs(a[m]-b[m]).max())
Z=3-b; c=m&(Z>1.03)
print('cup+handle footprint: X %.3f..%.3f  Y %.3f..%.3f  top z %.3f'%(X[c].min(),X[c].max(),Y[c].min(),Y[c].max(),Z[c].max()))
"
