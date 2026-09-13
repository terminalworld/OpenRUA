#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v parameter | head -40; echo ---; ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8 & sleep 4; kill %1 2>/dev/null; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 50)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(got.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["t"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
while not all(k in got for k in "dit"): rclpy.spin_once(node, timeout_sec=0.2)
d = np.frombuffer(got["d"].data, dtype=np.float32).reshape(got["d"].height, got["d"].width)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = got["t"]; q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.translation.x,t.translation.y,t.translation.z])
v,u = np.mgrid[0:d.shape[0],0:d.shape[1]]
P = np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1) @ R.T + T
np.save(f"{cam}_world.npy", P)
print("saved", P.shape)
EOF
python3 cloud.py birdview && python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
# print world coords at some pixels
for (u,v) in [(343,253),(343,240),(365,300),(415,305),(310,300),(230,265),(320,180),(100,100),(320,400)]:
    print((u,v), np.round(P[v,u],3))
EOF

# openrua op 8
cat /tmp/claude-1000/-workspace/ff964403-47bf-4887-9999-5a3fa9f24464/tasks/bg768l12p.output; ls -la /workspace/*.npy 2>/dev/null; ps aux | grep cloud.py | head

# openrua op 9
sed -i 's/from rclpy.qos import QoSProfile, DurabilityPolicy/from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy/; s/qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL)/qos = QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)/' cloud.py && sed -i 's/while not all(k in got for k in "dit"): rclpy.spin_once(node, timeout_sec=0.2)/import time\nt0=time.time()\nwhile not all(k in got for k in "dit"):\n    rclpy.spin_once(node, timeout_sec=0.2)\n    if time.time()-t0>20: raise SystemExit("missing: "+str([k for k in "dit" if k not in got]))/' cloud.py && timeout 90 python3 cloud.py birdview

# openrua op 10
sed -i 's|node.create_subscription(TFMessage, "/tf_static", cb, qos)|node.create_subscription(TFMessage, "/tf_static", cb, qos); node.create_subscription(TFMessage, "/tf", cb, 50)|' cloud.py && timeout 90 python3 cloud.py birdview && timeout 90 python3 cloud.py agentview && python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
for (u,v) in [(343,253),(343,240),(365,300),(415,305),(310,300),(230,265),(320,180),(100,100),(320,400)]:
    print((u,v), np.round(P[v,u],3))
EOF

# openrua op 11
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
z = P[...,2]
# height map grid over table region x in [-0.5,0.5], y in [-0.6,0.6], 2cm cells
xs = np.arange(-0.5,0.5,0.02); ys=np.arange(-0.6,0.6,0.02)
mask = (z>0.905)&(z<1.6)&(P[...,0]>-0.5)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
pts = P[mask]
print("pts above table:", len(pts))
# print coarse ascii height map, rows = x, cols = y
H = np.zeros((len(xs),len(ys)))
for p in pts:
    i=int((p[0]+0.5)/0.02); j=int((p[1]+0.6)/0.02)
    if 0<=i<len(xs) and 0<=j<len(ys): H[i,j]=max(H[i,j],p[2])
chars=" .:-=+*#%@"
for i,x in enumerate(xs):
    row="".join(chars[min(9,int((h-0.9)/0.05))] if h>0 else " " for h in H[i])
    print(f"{x:+.2f} {row}")
print("y axis: -0.6 .. 0.6")
EOF

# openrua op 12
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    P = np.load(f"{cam}_world.npy")
    z=P[...,2]; x=P[...,0]; y=P[...,1]
    print("==",cam)
    # cabinet body: tall region y>0.2
    m=(z>1.05)&(z<1.3)&(y>0.15)&(y<0.6)&(x>-0.3)&(x<0.3)
    if m.sum(): print("cabinet top: x",x[m].min().round(3),x[m].max().round(3)," y",y[m].min().round(3),y[m].max().round(3)," z",z[m].min().round(3),z[m].max().round(3), m.sum())
    # drawer region: y in [0,0.3], z between 0.91 and 1.05
    m=(z>0.91)&(z<1.05)&(y>-0.05)&(y<0.3)&(x>-0.2)&(x<0.2)
    if m.sum():
        print("drawer pts: x",x[m].min().round(3),x[m].max().round(3)," y",y[m].min().round(3),y[m].max().round(3)," z",z[m].min().round(3),z[m].max().round(3), m.sum())
        # histogram of z
        h,e=np.histogram(z[m],bins=14,range=(0.91,1.05)); print(" z hist",list(zip(e[:-1].round(3),h)))
        # for z near drawer rim (0.98-1.05) -> outline
        r=m&(z>0.99)
        if r.sum(): print(" rim: x",x[r].min().round(3),x[r].max().round(3)," y",y[r].min().round(3),y[r].max().round(3))
        f=m&(z<0.94)
        if f.sum(): print(" floor: x",x[f].min().round(3),x[f].max().round(3)," y",y[f].min().round(3),y[f].max().round(3), "zmean",z[f].mean().round(3))
    # bottle: near x -0.2..-0.1, y 0..0.15, exclude robot arm (robot is at x<-0.3 mostly)
    m=(z>0.92)&(z<1.35)&(y>0.0)&(y<0.2)&(x>-0.25)&(x<-0.05)
    if m.sum():
        print("bottle pts: x",x[m].min().round(3),x[m].max().round(3)," y",y[m].min().round(3),y[m].max().round(3)," z",z[m].min().round(3),z[m].max().round(3), m.sum())
        for zlo in [0.92,1.0,1.1,1.15,1.2,1.25]:
            s=m&(z>zlo)&(z<zlo+0.05)
            if s.sum(): print(f"  z{zlo:.2f}: x {x[s].mean():.3f} y {y[s].mean():.3f} n={s.sum()} xr={x[s].min():.3f}..{x[s].max():.3f} yr={y[s].min():.3f}..{y[s].max():.3f}")
    # bowl
    m=(z>0.92)&(z<1.0)&(y>-0.15)&(y<0.05)&(x>-0.1)&(x<0.1)
    if m.sum(): print("bowl: x",x[m].min().round(3),x[m].max().round(3)," y",y[m].min().round(3),y[m].max().round(3)," z",z[m].max().round(3))
EOF

# openrua op 13
timeout 90 python3 cloud.py robot0_robotview; timeout 90 python3 cloud.py sideview; python3 - <<'EOF'
import numpy as np
for cam in ["agentview","robot0_robotview","sideview"]:
    P = np.load(f"{cam}_world.npy")
    z=P[...,2]; x=P[...,0]; y=P[...,1]
    print("==",cam)
    m=(z>0.90)&(z<1.4)&(y>0.02)&(y<0.13)&(x>-0.19)&(x<-0.10)
    for zlo in np.arange(0.90,1.36,0.01):
        s=m&(z>=zlo)&(z<zlo+0.01)
        if s.sum()>3: print(f"  z{zlo:.2f}: n={s.sum():4d} x {x[s].min():.3f}..{x[s].max():.3f} (mean {x[s].mean():.3f})  y {y[s].min():.3f}..{y[s].max():.3f} (mean {y[s].mean():.3f})")
EOF

# openrua op 14
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
z=P[...,2]; x=P[...,0]; y=P[...,1]
m=(z>0.93)&(z<1.06)&(y>-0.1)&(y<0.25)&(x>-0.25)&(x<0.25)
print("x hist of drawer-height pts (rim/handle):")
h,e=np.histogram(x[m],bins=50,range=(-0.25,0.25))
for a,b in zip(e[:-1],h):
    if b: print(f"  x {a:+.2f}: {b}")
print("y hist:")
h,e=np.histogram(y[m],bins=35,range=(-0.1,0.25))
for a,b in zip(e[:-1],h):
    if b: print(f"  y {a:+.2f}: {b}")
# per z band, xy extents
for zlo in np.arange(0.92,1.06,0.01):
    s=m&(z>=zlo)&(z<zlo+0.01)
    if s.sum()>3: print(f"z{zlo:.2f} n={s.sum()} x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f}")
# floor region and its extents
f=(z>0.915)&(z<0.935)&(y>0.0)&(y<0.35)&(x>-0.2)&(x<0.2)
print("floor x",x[f].min(),x[f].max(),"y",y[f].min(),y[f].max())
# cabinet front face lower edge: points at y ~0.19 with z between 0.93 and 1.12?
c=(z>1.0)&(z<1.12)&(y>0.15)&(y<0.25)&(x>-0.15)&(x<0.15)
print("cabinet front-ish pts", c.sum(), "y", y[c].min() if c.sum() else None, y[c].max() if c.sum() else None)
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
for cam in ["agentview","robot0_robotview"]:
    P = np.load(f"{cam}_world.npy")
    z=P[...,2]; x=P[...,0]; y=P[...,1]
    print("==",cam)
    # cabinet front face region
    m=(x>-0.08)&(x<0.10)&(y>0.15)&(y<0.45)&(z>0.90)&(z<1.15)
    for zlo in np.arange(0.90,1.15,0.01):
        s=m&(z>=zlo)&(z<zlo+0.01)
        if s.sum()>3:
            print(f"  z{zlo:.2f}: n={s.sum():5d} y {y[s].min():.3f}..{y[s].max():.3f} (median {np.median(y[s]):.3f})")
EOF

# openrua op 16
python3 - <<'EOF'
import numpy as np
P = np.concatenate([np.load(f"{c}_world.npy").reshape(-1,3) for c in ["agentview","robot0_robotview","birdview","sideview"]])
x,y,z=P[:,0],P[:,1],P[:,2]
m=(np.abs(x-0.03)<0.02)&(y>-0.12)&(y<0.45)&(z>0.89)&(z<1.15)&np.isfinite(z)
ys=np.arange(-0.12,0.45,0.01); zs=np.arange(1.15,0.89,-0.005)
G=np.zeros((len(zs),len(ys)),int)
for yy,zz in zip(y[m],z[m]):
    j=int((yy+0.12)/0.01); i=int((1.15-zz)/0.005)
    if 0<=i<len(zs) and 0<=j<len(ys): G[i,j]+=1
print("y-z profile at x≈0.03 (cols y from -0.12 step 0.01; rows z from 1.15 down step 0.005)")
print("      "+"".join(str(int(round(yy*100))%10) for yy in ys))
for i,zz in enumerate(zs):
    print(f"{zz:.3f} "+"".join("#" if g>20 else ("+" if g>3 else ".") if g>0 else " " for g in G[i]))
EOF

# openrua op 17
python3 - <<'EOF'
import numpy as np
P = np.concatenate([np.load(f"{c}_world.npy").reshape(-1,3) for c in ["agentview","robot0_robotview","birdview","sideview"]])
x,y,z=P[:,0],P[:,1],P[:,2]
for yc in [0.15, 0.30]:
    m=(np.abs(y-yc)<0.02)&(x>-0.25)&(x<0.25)&(z>0.89)&(z<1.15)&np.isfinite(z)
    xs=np.arange(-0.25,0.25,0.01); zs=np.arange(1.15,0.89,-0.005)
    G=np.zeros((len(zs),len(xs)),int)
    for xx,zz in zip(x[m],z[m]):
        j=int((xx+0.25)/0.01); i=int((1.15-zz)/0.005)
        if 0<=i<len(zs) and 0<=j<len(xs): G[i,j]+=1
    print(f"x-z profile at y≈{yc} (cols x from -0.25 step 0.01)")
    print("      "+"".join(str(int(round(xx*100))%10) for xx in xs))
    for i,zz in enumerate(zs):
        print(f"{zz:.3f} "+"".join("#" if g>20 else ("+" if g>3 else ".") if g>0 else " " for g in G[i]))
EOF

# openrua op 18
python3 - <<'EOF'
import numpy as np
# tray interior (x0,x1,y0,y1)
X0,X1,Y0,Y1=-0.085,0.115,0.08,0.23
def footprint(cx,cy,th):
    # bottle axis direction d, base end at s=-0.075.. body to +0.075, neck to +0.155 ; center of body = (cx,cy)
    d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    pts=[]
    for s,r in [(-0.075,0.021),(0.075,0.021),(0.095,0.008),(0.155,0.008)]:
        for sgn in (-1,1):
            pts.append(np.array([cx,cy])+s*d+sgn*r*n)
    return np.array(pts)
best=None
for th in np.radians(np.arange(20,70,1)):
    for cx in np.arange(X0,X1,0.005):
        for cy in np.arange(Y0,Y1,0.005):
            p=footprint(cx,cy,th)
            margin=min(p[:,0].min()-X0, X1-p[:,0].max(), p[:,1].min()-Y0, Y1-p[:,1].max())
            if best is None or margin>best[0]: best=(margin,np.degrees(th),cx,cy)
print("best margin %.4f at theta=%.0f deg, body center (%.3f,%.3f)"%best)
m,th,cx,cy=best
print(footprint(cx,cy,np.radians(th)).round(3))
EOF

# openrua op 19
find / -iname "*wine*" 2>/dev/null | head; find / -iname "*cabinet*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 4 2>/dev/null | head; pip list 2>/dev/null | grep -i -E "libero|robosuite|mujoco"

# openrua op 20
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>/dev/null | head -c 3000; echo; ros2 topic echo /pipeline_state --once 2>&1 | head -20; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 21
cat > /workspace/rob.py <<'EOF'
"""Reusable robot helper: FK/IK, trajectory, gripper, joint state."""
import math, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (identity rotation)
TCP = M["hand"]["tcp_offset_m"]

class Robot(Node):
    def __init__(self):
        super().__init__("rob")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        while self.js is None: rclpy.spin_once(self, timeout_sec=0.2)

    def _js(self, m): self.js = m

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None: rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS], d

    def finger_gap(self):
        _, d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def _spin(self, fut, timeout=600):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def fk_world(self, q=None):
        """hand pose in world: (pos[3], quat xyzw)."""
        if q is None: q, _ = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        res = self._spin(self.fk.call_async(req), 60)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_world(self, pos, quat, seed=None, at_tcp=True):
        """IK for hand pose in world (pos of TCP if at_tcp). returns joint list or None."""
        pos = np.array(pos, float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        if seed is None: seed, _ = self.joints()
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = list(seed)
        res = self._spin(self.ik.call_async(req), 60)
        if res is None or res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in JOINTS]

    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint lists; seconds: total or list of cumulative times."""
        if isinstance(seconds, (int, float)):
            n = len(waypoints); seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        gh = self._spin(self.traj.send_goal_async(goal), 60)
        res = self._spin(gh.get_result_async(), 900)
        code = res.result.error_code
        q, _ = self.joints()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        gh = self._spin(self.grip.send_goal_async(g), 60)
        res = self._spin(gh.get_result_async(), 300)
        return res.result.reached_goal, res.result.stalled, self.finger_gap()

def quat_down(yaw_deg=0.0):
    """hand z pointing down (world -z), fingers closing along world axis rotated by yaw about z.
    yaw=0: hand y axis along world y."""
    R = Rot.from_euler("xyz", [180, 0, yaw_deg], degrees=True)
    return R.as_quat()

def init():
    rclpy.init()
    return Robot()
EOF
timeout 120 python3 -c "
import rob, numpy as np
r = rob.init()
q,d = r.joints()
print('q', np.round(q,4)); print('fingers', d['panda_finger_joint1'], d['panda_finger_joint2'])
pos, quat = r.fk_world()
print('hand world pos', pos.round(4), 'quat', quat.round(4))
from scipy.spatial.transform import Rotation as Rot
print('hand R:\n', Rot.from_quat(quat).as_matrix().round(3))
print('TCP world', (pos + rob.TCP*Rot.from_quat(quat).as_matrix()[:,2]).round(4))
"

# openrua op 22
timeout 300 python3 -c "
import rob, numpy as np
rob.BASE = np.zeros(3)
r = rob.init()
q,_ = r.joints()
pos, quat = r.fk_world()
print('hand world (raw FK)', pos.round(4))
sol = r.ik_world(pos, quat, seed=q, at_tcp=False)
print('IK raw-world ->', None if sol is None else np.round(sol,3))
sol2 = r.ik_world(pos - np.array([-0.66,0,0.912]), quat, seed=q, at_tcp=False)
print('IK base-frame ->', None if sol2 is None else np.round(sol2,3))
"

# openrua op 23
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.zeros(3)  # FK\/IK services already answer in the world frame (verified)/' rob.py && timeout 300 python3 -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
pos, quat = r.fk_world()
sol = r.ik_world(pos, quat, seed=q, at_tcp=False)
p2,q2 = r.fk_world(sol)
print('orig', pos.round(4), quat.round(4)); print('sol ', p2.round(4), q2.round(4), np.round(sol,3))
"

# openrua op 24
timeout 600 python3 -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
q,_ = r.joints()
for yaw in [0, 45, 90, -45]:
    quat = rob.quat_down(yaw)
    tgt = np.array([-0.15, 0.07, 1.15])
    for trial in range(3):
        sol = r.ik_world(tgt, quat, seed=q, at_tcp=True)
        if sol is None: print('yaw',yaw,'trial',trial,'IK None'); continue
        p2,q2 = r.fk_world(sol)
        tcp = p2 + rob.TCP*Rot.from_quat(q2).as_matrix()[:,2]
        ang = (Rot.from_quat(q2).inv()*Rot.from_quat(quat)).magnitude()*180/np.pi
        print('yaw',yaw,'trial',trial,'tcp err', np.round(tcp-tgt,4), 'ang err deg', round(ang,2), 'q', np.round(sol,2))
"

# openrua op 25
python3 - <<'EOF'
import re
s = open("/workspace/rob.py").read()
s = s.replace("""        pos = pos - BASE
        req = GetPositionIK.Request()""", """        pos = pos - BASE
        # the IK tip is panda_link8 = panda_hand rotated +45deg about z (verified via FK)
        quat = (Rot.from_quat(quat) * Rot.from_euler("z", 45, degrees=True)).as_quat()
        req = GetPositionIK.Request()""")
open("/workspace/rob.py","w").write(s)
EOF
timeout 600 python3 -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
q,_ = r.joints()
for yaw in [0, 45, 90]:
    quat = rob.quat_down(yaw)
    tgt = np.array([-0.15, 0.07, 1.15])
    sol = r.ik_world(tgt, quat, seed=q, at_tcp=True)
    if sol is None: print('yaw',yaw,'IK None'); continue
    p2,q2 = r.fk_world(sol)
    tcp = p2 + rob.TCP*Rot.from_quat(q2).as_matrix()[:,2]
    ang = (Rot.from_quat(q2).inv()*Rot.from_quat(quat)).magnitude()*180/np.pi
    print('yaw',yaw,'tcp err', np.round(tcp-tgt,4), 'ang err deg', round(ang,2), 'q', np.round(sol,3))
"

# openrua op 26
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
tgt = np.array([-0.04, 0.15, 1.38])
sol = r.ik_world(tgt, rob.quat_down(0), seed=q, at_tcp=False)  # hand frame at 1.38 -> camera looks down at tray
print('sol', None if sol is None else np.round(sol,3))
code, err = r.move_joints([sol], 4.0)
print('move code', code, 'max joint err', round(err,4))
pos, quat = r.fk_world(); print('hand now', pos.round(4), quat.round(3))
" && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 27
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_eye_in_hand_world.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
# floor of tray: z in 0.92-0.935
f=(z>0.918)&(z<0.935)&(x>-0.2)&(x<0.2)&(y>0.0)&(y<0.3)
print("floor pts", f.sum(), "x", x[f].min().round(4), x[f].max().round(4), "y", y[f].min().round(4), y[f].max().round(4), "z", z[f].mean().round(4))
# wall tops z 0.975-0.995
w=(z>0.975)&(z<0.995)&(x>-0.2)&(x<0.2)&(y>0.0)&(y<0.3)
print("wall-top pts", w.sum())
h,e=np.histogram(x[w],bins=80,range=(-0.2,0.2))
print("x hist wall tops:", [(round(a,3),b) for a,b in zip(e[:-1],h) if b>5])
h,e=np.histogram(y[w],bins=60,range=(0.0,0.3))
print("y hist wall tops:", [(round(a,3),b) for a,b in zip(e[:-1],h) if b>5])
# floor extents by row: for y bands, x range of floor
for yc in np.arange(0.09,0.24,0.02):
    s=f&(np.abs(y-yc)<0.005)
    if s.sum(): print(f" y={yc:.2f}: floor x {x[s].min():.4f}..{x[s].max():.4f}")
for xc in np.arange(-0.08,0.12,0.02):
    s=f&(np.abs(x-xc)<0.005)
    if s.sum(): print(f" x={xc:.2f}: floor y {y[s].min():.4f}..{y[s].max():.4f}")
# bottle from this view
b=(z>0.95)&(z<1.2)&(x>-0.2)&(x<-0.1)&(y>0.03)&(y<0.12)
print("bottle top pts", b.sum(), "zmax", z[b].max().round(4))
for zlo in np.arange(0.95,1.14,0.01):
    s=b&(z>=zlo)&(z<zlo+0.01)
    if s.sum()>3: print(f"  z{zlo:.2f}: n={s.sum():4d} x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f}")
EOF

# openrua op 28
python3 - <<'EOF'
import numpy as np
X0,X1,Y0,Y1=-0.089,0.119,0.0855,0.2385
def footprint(cx,cy,th,rb=0.02,rn=0.008,Lb=0.145,Ln=0.08):
    d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    pts=[]
    for s,r in [(-Lb/2,rb),(Lb/2,rb),(Lb/2+0.02,rn),(Lb/2+Ln,rn)]:
        for sgn in (-1,1):
            pts.append(np.array([cx,cy])+s*d+sgn*r*n)
    return np.array(pts)
for rb in [0.019,0.02,0.021]:
    best=None
    for th in np.radians(np.arange(15,75,0.5)):
        for cx in np.arange(X0,X1,0.0025):
            for cy in np.arange(Y0,Y1,0.0025):
                p=footprint(cx,cy,th,rb)
                margin=min(p[:,0].min()-X0, X1-p[:,0].max(), p[:,1].min()-Y0, Y1-p[:,1].max())
                if best is None or margin>best[0]: best=(margin,np.degrees(th),cx,cy)
    print("rb=%.3f best margin %.4f at theta=%.1f deg, body center (%.4f,%.4f)"%((rb,)+best))
EOF

# openrua op 29
timeout 60 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node("rd")
got=[]
n.create_subscription(String,"/robot_description",lambda m: got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.2)
open("/workspace/robot.urdf","w").write(got[0] if got else "")
print(len(got[0]) if got else "none")
EOF
grep -n -A12 '<link name="panda_hand"' /workspace/robot.urdf | head -40; grep -n -A12 '<link name="panda_leftfinger"' /workspace/robot.urdf | head -30; grep -n "mesh filename" /workspace/robot.urdf | head -3

# openrua op 30
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ && python3 - <<'EOF'
import struct, numpy as np
def stl_bounds(path):
    with open(path,'rb') as f:
        data=f.read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        vs=[]
        for line in data.decode(errors='ignore').splitlines():
            line=line.strip()
            if line.startswith('vertex'):
                vs.append([float(v) for v in line.split()[1:4]])
        v=np.array(vs)
    else:
        n=struct.unpack_from('<I',data,80)[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for name in ["hand","finger","link7","link6"]:
    lo,hi=stl_bounds(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{name}.stl")
    print(name, "min",lo.round(4),"max",hi.round(4))
EOF

# openrua op 31
find / -name "hand.stl" -o -name "hand.dae" -o -name "finger.stl" 2>/dev/null | head

# openrua op 32
python3 - <<'EOF'
import numpy as np
X0,X1,Y0,Y1=-0.089,0.119,0.0855,0.2385
def footprint(cx,cy,th,rb=0.02,rn=0.008,Lb=0.145,Ln=0.08):
    d=np.array([np.cos(th),np.sin(th)]); n=np.array([-d[1],d[0]])
    pts=[]
    for s,r in [(-Lb/2,rb),(Lb/2,rb),(Lb/2+0.02,rn),(Lb/2+Ln,rn)]:
        for sgn in (-1,1):
            pts.append(np.array([cx,cy])+s*d+sgn*r*n)
    return np.array(pts)
best=None
for th in np.radians(np.arange(110,170,0.5)):
    for cx in np.arange(X0,X1,0.0025):
        for cy in np.arange(Y0,Y1,0.0025):
            p=footprint(cx,cy,th)
            margin=min(p[:,0].min()-X0, X1-p[:,0].max(), p[:,1].min()-Y0, Y1-p[:,1].max())
            if best is None or margin>best[0]: best=(margin,np.degrees(th),cx,cy)
print("best margin %.4f at theta=%.1f deg, body center (%.4f,%.4f)"%best)
m,th,cx,cy=best; th=np.radians(th)
d=np.array([np.cos(th),np.sin(th)])
print("base end", (np.array([cx,cy])-0.0725*d).round(4), "neck tip", (np.array([cx,cy])+0.1525*d).round(4), "neck grasp pt (2cm from tip)", (np.array([cx,cy])+0.1325*d).round(4))
EOF

# openrua op 33
cat >> /workspace/rob.py <<'EOF'

# ---- task-specific frames ----
def R_final():
    """hand orientation for the horizontal bottle: hand z (palm->fingertips) = from neck toward base
    along the tray diagonal, fingers pinching vertically (hand y = world z)."""
    zf = np.array([0.834, -0.552, 0.0]); zf /= np.linalg.norm(zf)
    yf = np.array([0.0, 0.0, 1.0])
    xf = np.cross(yf, zf)
    return np.column_stack([xf, yf, zf])

def R_grasp():
    """top-down neck grasp orientation such that a 90deg rotation about a horizontal axis
    brings it to R_final (bottle swings from vertical to the diagonal)."""
    Rf = R_final()
    zg = np.array([0.0, 0.0, -1.0]); zf = Rf[:, 2]
    k = np.cross(zg, zf); k /= np.linalg.norm(k)
    Rrot = Rot.from_rotvec(k * np.pi / 2).as_matrix()
    Rg = Rrot.T @ Rf
    assert np.allclose(Rg[:, 2], zg, atol=1e-6), Rg
    return Rg

def R_mid(frac):
    """interpolated orientation between R_grasp and R_final (frac in [0,1])."""
    Rf = R_final(); zg = np.array([0.0, 0.0, -1.0]); zf = Rf[:, 2]
    k = np.cross(zg, zf); k /= np.linalg.norm(k)
    Rrot = Rot.from_rotvec(k * np.pi / 2 * frac).as_matrix()
    return Rrot @ R_grasp()

def q_of(R): return Rot.from_matrix(R).as_quat()
EOF
timeout 300 python3 -c "
import rob, numpy as np
print('R_final\n', rob.R_final().round(3)); print('R_grasp\n', rob.R_grasp().round(3))
print('R_mid(1) == R_final?', np.allclose(rob.R_mid(1), rob.R_final()))
r = rob.init()
q,_ = r.joints()
# IK feasibility checks for the key poses
poses = {
 'above_bottle': ([-0.147, 0.075, 1.25], rob.q_of(rob.R_grasp())),
 'grasp_neck':   ([-0.147, 0.075, 1.10], rob.q_of(rob.R_grasp())),
 'lift':         ([-0.147, 0.075, 1.35], rob.q_of(rob.R_grasp())),
 'rot_half':     ([-0.147, 0.075, 1.35], rob.q_of(rob.R_mid(0.5))),
 'rotated':      ([-0.147, 0.075, 1.35], rob.q_of(rob.R_final())),
 'above_place':  ([-0.0645, 0.2186, 1.25], rob.q_of(rob.R_final())),
 'place':        ([-0.0645, 0.2186, 1.01], rob.q_of(rob.R_final())),
}
seed = q
for name,(p,qq) in poses.items():
    sol = r.ik_world(p, qq, seed=seed, at_tcp=True)
    print(name, None if sol is None else np.round(sol,3))
    if sol is not None: seed = sol
"

# openrua op 34
cat >> /workspace/rob.py <<'EOF'

LINKS = ["panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]
def fk_links(self, q):
    req = GetPositionFK.Request()
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = JOINTS
    req.robot_state.joint_state.position = list(q)
    res = self._spin(self.fk.call_async(req), 60)
    return {n: np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for n, p in zip(LINKS, res.pose_stamped)}
Robot.fk_links = fk_links

def ik_best(self, pos, quat, seed, tries=8, at_tcp=True, max_step=None):
    """IK with several seeds (given seed + jittered); returns solution closest to seed in joint space."""
    rng = np.random.default_rng(0)
    best = None
    for i in range(tries):
        s = list(seed) if i == 0 else list(np.clip(np.array(seed) + rng.normal(0, 0.3 if i < 4 else 0.8, 7),
                                               [l[0] for l in ARM["limits_rad"]], [l[1] for l in ARM["limits_rad"]]))
        sol = self.ik_world(pos, quat, seed=s, at_tcp=at_tcp)
        if sol is None: continue
        d = max(abs(a - b) for a, b in zip(sol, seed))
        if best is None or d < best[0]: best = (d, sol)
    if best is None: return None
    if max_step is not None and best[0] > max_step: return None
    return best[1]
Robot.ik_best = ik_best
EOF
timeout 900 python3 -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
seed = r.ik_best([-0.147, 0.075, 1.35], rob.q_of(rob.R_grasp()), q)
print('lift', np.round(seed,3))
path=[seed]
for frac in np.linspace(0.1,1.0,10):
    sol = r.ik_best([-0.147, 0.075, 1.35], rob.q_of(rob.R_mid(frac)), path[-1], tries=12)
    if sol is None: print('frac',frac,'FAILED'); break
    d = max(abs(a-b) for a,b in zip(sol,path[-1]))
    links = r.fk_links(sol)
    zmin = min(v[2] for v in links.values())
    print(f'frac {frac:.1f} step {d:.3f} q {np.round(sol,2)} zmin {zmin:.3f} link4 {links[\"panda_link4\"].round(2)} link6 {links[\"panda_link6\"].round(2)}')
    path.append(sol)
"

# openrua op 35
timeout 900 python3 -c "
import rob, numpy as np
r = rob.init()
qrot = [0.36,-0.34,0.06,-2.55,-1.92,2.03,1.51]
qf = rob.q_of(rob.R_final())
prev = qrot
for name,p in [('above_place_hi',[-0.0645,0.2186,1.35]),('above_place',[-0.0645,0.2186,1.25]),('mid',[-0.0645,0.2186,1.12]),('place',[-0.0645,0.2186,1.01])]:
    sol = r.ik_best(p, qf, prev, tries=12)
    if sol is None: print(name,'FAILED'); break
    d = max(abs(a-b) for a,b in zip(sol,prev))
    links = r.fk_links(sol)
    zmin = min(v[2] for v in links.values())
    print(f'{name} step {d:.3f} q {np.round(sol,2)} zmin {zmin:.3f} l6 {links[\"panda_link6\"].round(3)} l7 {links[\"panda_link7\"].round(3)} hand {links[\"panda_hand\"].round(3)}')
    prev = sol
"

# openrua op 36
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
print('gripper open:', r.gripper(0.04))
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
s1 = r.ik_best([-0.147, 0.075, 1.25], qg, q)
print('above q', np.round(s1,3))
print('move above:', r.move_joints([s1], 4.0))
s2 = r.ik_best([-0.147, 0.075, 1.10], qg, s1)
print('descend:', r.move_joints([s2], 2.5))
pos, quat = r.fk_world(); from scipy.spatial.transform import Rotation as Rot
print('TCP now', (pos + rob.TCP*Rot.from_quat(quat).as_matrix()[:,2]).round(4))
" && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 37
timeout 120 python3 tools/perception/cam_snap.py frontview && timeout 120 python3 tools/perception/cam_snap.py sideview

# openrua op 38
timeout 600 python3 -u -c "
import rob
r = rob.init()
print('close:', r.gripper(0.0))
_, d = r.joints()
print('fingers', d['panda_finger_joint1'], d['panda_finger_joint2'], 'gap', r.finger_gap())
"

# openrua op 39
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
s = r.ik_best([-0.147, 0.075, 1.20], qg, q)
print('lift 10cm:', r.move_joints([s], 2.5), 'gap', r.finger_gap())
" && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 40
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
print('open:', r.gripper(0.04))
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
s = r.ik_best([-0.147, 0.075, 1.075], qg, q)
print('descend:', r.move_joints([s], 2.5))
print('close:', r.gripper(0.0))
s = r.ik_best([-0.147, 0.075, 1.20], qg, s)
print('lift:', r.move_joints([s], 2.5), 'gap', r.finger_gap())
" && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 41
timeout 120 python3 cloud.py frontview && python3 - <<'EOF'
import numpy as np
P=np.load("frontview_world.npy"); x,y,z=P[...,0],P[...,1],P[...,2]
m=(x>-0.2)&(x<-0.1)&(y>0.03)&(y<0.12)&(z>0.9)&(z<1.25)
print("bottle-ish pts", m.sum(), "zmax", z[m].max())
for zlo in np.arange(0.90,1.2,0.01):
    s=m&(z>=zlo)&(z<zlo+0.01)
    if s.sum()>2: print(f"  z{zlo:.2f}: n={s.sum():4d} x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f}")
# pixel of top
vv,uu=np.where(m&(z>z[m].max()-0.01)); print("top pixels v,u:", vv.mean(), uu.mean())
EOF

# openrua op 42
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
i=s.index("# ---- task-specific frames ----")
s=s[:i]+'''# ---- task-specific frames ----
TH = np.radians(33.5)
D = np.array([np.cos(TH), np.sin(TH), 0.0])      # bottle axis, neck -> base, when lying in the tray
N = np.array([-np.sin(TH), np.cos(TH), 0.0])     # horizontal, perpendicular to D (finger closing axis)

def R_final():
    """hand orientation for the horizontal bottle: hand z (palm->fingertips) = D (from neck toward base),
    fingers pinching horizontally along N; hand x = y cross z (vertical)."""
    zf = D; yf = N; xf = np.cross(yf, zf)
    return np.column_stack([xf, yf, zf])

def R_grasp():
    """top-down neck grasp orientation: 90deg swing about the horizontal axis brings it to R_final."""
    return R_mid(0.0)

def R_mid(frac):
    Rf = R_final(); zg = np.array([0.0, 0.0, -1.0]); zf = Rf[:, 2]
    k = np.cross(zg, zf); k /= np.linalg.norm(k)
    Rrot_full = Rot.from_rotvec(k * np.pi / 2).as_matrix()
    Rg = Rrot_full.T @ Rf
    assert np.allclose(Rg[:, 2], zg, atol=1e-6)
    return Rot.from_rotvec(k * np.pi / 2 * frac).as_matrix() @ Rg

def q_of(R): return Rot.from_matrix(R).as_quat()
'''+s[s.index("LINKS = ["):]
open("/workspace/rob.py","w").write(s)
EOF
timeout 1500 python3 -u -c "
import rob, numpy as np
print('R_grasp\n', rob.R_grasp().round(3)); print('R_final\n', rob.R_final().round(3))
r = rob.init()
print('open:', r.gripper(0.04))
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
s = r.ik_best([-0.147, 0.075, 1.15], qg, q)
print('above:', r.move_joints([s], 3.0))
s = r.ik_best([-0.147, 0.075, 1.02], qg, s)
print('descend:', r.move_joints([s], 2.5))
print('close:', r.gripper(0.0))
s = r.ik_best([-0.147, 0.075, 1.10], qg, s)
print('lift 8cm:', r.move_joints([s], 2.5), 'gap', r.finger_gap())
" && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 43
cat >> /workspace/rob.py <<'EOF'

def go(self, q_target, seconds=None, tol=0.02, attempts=5):
    """move to joint target; re-send until converged (controller lag gives -5 on long goals)."""
    q_target = [float(v) for v in q_target]
    for i in range(attempts):
        q, _ = self.joints()
        dist = max(abs(a - b) for a, b in zip(q, q_target))
        if dist < tol: return True, dist
        t = seconds if seconds is not None else max(2.0, min(10.0, dist * 4.0))
        code, err = self.move_joints([q_target], t)
        print(f"  go: attempt {i} dist {dist:.3f} t {t:.1f}s -> code {code} err {err:.4f}")
        if err < tol: return True, err
    return False, err
Robot.go = go
EOF
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints(); print('q now', np.round(q,3))
qg = rob.q_of(rob.R_grasp())
s = r.ik_best([-0.147, 0.075, 1.15], qg, q)
print('target', np.round(s,3))
print('above:', r.go(s))
pos, quat = r.fk_world(); from scipy.spatial.transform import Rotation as Rot
print('TCP', (pos + rob.TCP*Rot.from_quat(quat).as_matrix()[:,2]).round(4), 'R\n', Rot.from_quat(quat).as_matrix().round(2))
"

# openrua op 44
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
home = [0, -0.16, 0, -2.44, 0, 2.23, 0.785]
s_up = r.ik_best([-0.147, 0.075, 1.35], qg, q)
print('up:', r.go(s_up))
s_nat = r.ik_best([-0.147, 0.075, 1.35], qg, home, tries=16)
print('natural sol', np.round(s_nat,3), 'links', {k:v.round(2) for k,v in r.fk_links(s_nat).items() if k in ('panda_link4','panda_link6','panda_link7')})
print('to natural:', r.go(s_nat, seconds=6.0))
s = r.ik_best([-0.147, 0.075, 1.02], qg, s_nat)
print('descend sol', np.round(s,3))
print('descend:', r.go(s))
pos, quat = r.fk_world(); from scipy.spatial.transform import Rotation as Rot
print('TCP', (pos + rob.TCP*Rot.from_quat(quat).as_matrix()[:,2]).round(4))
" && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 45
cat >> /workspace/rob.py <<'EOF'

def ik_natural(self, pos, quat, at_tcp=True, tries=24, seed_hint=None, verbose=False):
    """sample many IK solutions and pick a 'natural' one: shoulder pointing at target (j1 ~ azimuth),
    j3 ~ 0, j5 ~ 0, elbow up; all wrist links above the table."""
    lim = np.array(ARM["limits_rad"])
    pos = np.array(pos, float)
    az = np.arctan2(pos[1] - (-0.0), pos[0] - (-0.66))
    rng = np.random.default_rng(1)
    cands = []
    seeds = [[az, -0.3, 0.0, -2.0, 0.0, 1.8, 0.785], [az, 0.3, 0.0, -1.8, 0.0, 2.2, 0.785],
             [az, -0.16, 0.0, -2.44, 0.0, 2.23, 0.785], [az, 0.6, 0.0, -1.5, 0.0, 2.1, 0.0]]
    if seed_hint is not None: seeds.insert(0, list(seed_hint))
    for i in range(tries):
        if i < len(seeds): s = seeds[i]
        else:
            b = seeds[i % len(seeds)]
            s = list(np.clip(np.array(b) + rng.normal(0, 0.5, 7), lim[:, 0], lim[:, 1]))
        sol = self.ik_world(pos, quat, seed=s, at_tcp=at_tcp)
        if sol is None: continue
        links = self.fk_links(sol)
        zmin = min(links[k][2] for k in ("panda_link5", "panda_link6", "panda_link7", "panda_hand"))
        if zmin < 1.0: continue
        cost = 2 * abs(sol[0] - az) + abs(sol[2]) + abs(sol[4]) + 0.3 * abs(sol[6] - 0.785)
        if seed_hint is not None: cost += 0.5 * max(abs(a - b) for a, b in zip(sol, seed_hint))
        cands.append((cost, sol))
    if not cands: return None
    cands.sort(key=lambda c: c[0])
    if verbose:
        for c, s in cands[:5]: print(f"   cand cost {c:.2f} q {np.round(s,2)}")
    return cands[0][1]
Robot.ik_natural = ik_natural
EOF
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
q,_ = r.joints()
qg = rob.q_of(rob.R_grasp())
s_up = r.ik_best([-0.147, 0.075, 1.30], qg, q)
print('up:', r.go(s_up))
s_nat = r.ik_natural([-0.147, 0.075, 1.30], qg, verbose=True)
print('natural', np.round(s_nat,3))
" && timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 46
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace("    zf = D; yf = N; xf = np.cross(yf, zf)","    zf = D; yf = -N; xf = np.cross(yf, zf)   # fingers symmetric: sign chosen to keep j7 away from its limit")
open("/workspace/rob.py","w").write(s)
EOF
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
print('R_grasp\n', rob.R_grasp().round(3)); print('R_final\n', rob.R_final().round(3))
qg = rob.q_of(rob.R_grasp())
print('grasp pose candidates:')
s_nat = r.ik_natural([-0.147, 0.075, 1.30], qg, verbose=True)
print('final pose candidates:')
qf = rob.q_of(rob.R_final())
s_fin = r.ik_natural([-0.033, 0.130, 0.97], qf, verbose=True)
print('links at final:', {k:v.round(3) for k,v in r.fk_links(s_fin).items()})
"

# openrua op 47
python3 - <<'EOF'
s=open("/workspace/rob.py").read()
s=s.replace('''        zmin = min(links[k][2] for k in ("panda_link5", "panda_link6", "panda_link7", "panda_hand"))
        if zmin < 1.0: continue''','''        if min(links[k][2] for k in ("panda_link5", "panda_link6")) < 1.0: continue
        if min(links[k][2] for k in ("panda_link7", "panda_hand")) < 0.95: continue''')
open("/workspace/rob.py","w").write(s)
EOF
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
qf = rob.q_of(rob.R_final())
print('final pose candidates:')
s_fin = r.ik_natural([-0.033, 0.130, 0.97], qf, verbose=True)
print('links at final:', {k:v.round(3) for k,v in r.fk_links(s_fin).items()})
print('rotated-at-height candidates:')
s_rot = r.ik_natural([-0.147, 0.075, 1.25], qf, verbose=True)
"

# openrua op 48
timeout 1500 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
Rf2 = rob.R_final() @ Rot.from_euler('z', 180, degrees=True).as_matrix()
print('R_final flipped\n', Rf2.round(3))
qf2 = rob.q_of(Rf2)
print('final flipped candidates:')
s = r.ik_natural([-0.033, 0.130, 0.97], qf2, verbose=True)
if s is not None: print('links:', {k:v.round(3) for k,v in r.fk_links(s).items() if k in ('panda_link4','panda_link5','panda_link6','panda_link7','panda_hand')})
print('rotated-at-height flipped candidates:')
s = r.ik_natural([-0.147, 0.075, 1.25], qf2, verbose=True)
"

# openrua op 49
ls *.npy && python3 -c "
import numpy as np
for cam in ['frontview','agentview','sideview','birdview']:
    try: P=np.load(f'{cam}_world.npy').reshape(-1,3)
    except Exception as e: print(cam,e); continue
    P=P[np.isfinite(P).all(1)]
    # cabinet region y>0.23
    c=P[(P[:,1]>0.23)&(P[:,2]>0.95)]
    print(cam,'cabinet pts',len(c), 'zmax',c[:,2].max() if len(c) else None, 'x range', (c[:,0].min(),c[:,0].max()) if len(c) else None, 'ymin', c[:,1].min() if len(c) else None)
    # anything high above z 1.1 near the workspace
    h=P[(P[:,2]>1.1)&(np.abs(P[:,0])<0.4)&(P[:,1]>-0.3)&(P[:,1]<0.6)]
    print('  high pts',len(h))
    if len(h):
        for z0 in np.arange(1.1,1.6,0.1):
            s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.1)]
            if len(s): print(f'   z {z0:.1f}-{z0+0.1:.1f}: n={len(s)} x[{s[:,0].min():.2f},{s[:,0].max():.2f}] y[{s[:,1].min():.2f},{s[:,1].max():.2f}]')
"


# openrua op 50
python3 -c "
import numpy as np
for cam in ['frontview','agentview','sideview','birdview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    s=P[(P[:,2]>0.91)&(P[:,2]<1.1)&(P[:,0]>-0.32)&(P[:,0]<-0.04)&(P[:,1]>-0.2)&(P[:,1]<0.12)]
    print(cam,len(s))
    if len(s):
        # 2cm grid occupancy
        g=np.floor((s[:,:2]-[-0.32,-0.2])/0.02).astype(int)
        H=np.zeros((14,16),int)
        for a,b in g: H[a,b]+=1
        print('   rows x from -0.32 step 0.02 ; cols y from -0.2 step 0.02')
        for i,row in enumerate(H): print(f'   x={-0.32+i*0.02:+.2f} ', ''.join('#' if v>20 else ('.' if v>0 else ' ') for v in row))
"


# openrua op 51
timeout 600 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
q,_ = r.joints(); print('q now', np.round(q,3)); p,qq = r.fk_world(); print('hand', p.round(3), 'tcp z', (p[2]-rob.TCP*Rot.from_quat(qq).as_matrix()[2,2]).round(3))
print('gap', r.finger_gap())
D = np.array(rob.D); k = np.cross([0,0,-1.0], D); k/=np.linalg.norm(k)
Rg = rob.R_grasp()
def R_tilt(th_deg, spin=0):
    return Rot.from_rotvec(k*np.radians(th_deg)).as_matrix() @ Rg @ Rot.from_euler('z', spin, degrees=True).as_matrix()
th = 55  # 90-35: bottle axis tilted 35 deg below horizontal
R = R_tilt(th)
print('R_tilt55\n', R.round(3))
base = np.array([0.06,0.19,0.955])
tcp = base - 0.115*R[:,2]
print('tcp target', tcp.round(4), 'palm', (tcp-rob.TCP*R[:,2]).round(4))
for spin in (0,180):
    print('spin',spin)
    s = r.ik_natural(tcp, rob.q_of(R_tilt(th,spin)), verbose=True)
    if s is not None:
        L = r.fk_links(s); print('  links', {kk:v.round(3) for kk,v in L.items() if kk in ('panda_link5','panda_link7','panda_hand')})
"

# openrua op 52
timeout 600 python3 -u -c "
import rob
r = rob.init()
print('open ->', r.gripper(0.04))
print('gap', r.finger_gap())
" && timeout 120 python3 cloud.py frontview && python3 -c "
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
b=P[(P[:,0]>-0.20)&(P[:,0]<-0.10)&(P[:,1]>0.02)&(P[:,1]<0.13)&(P[:,2]>0.91)&(P[:,2]<1.12)]
print('bottle pts',len(b),'z range',b[:,2].min().round(3),b[:,2].max().round(3))
top=b[b[:,2]>b[:,2].max()-0.03]; print('top xy mean',top[:,:2].mean(0).round(3))
mid=b[(b[:,2]>0.95)&(b[:,2]<1.0)]; print('body x range',mid[:,0].min().round(3),mid[:,0].max().round(3),'y range',mid[:,1].min().round(3),mid[:,1].max().round(3))
"

# openrua op 53
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.init()
Rg = rob.R_grasp(); qg = rob.q_of(Rg)
s = r.ik_natural([-0.147,0.075,1.15], qg, verbose=True)
print('pregrasp q', np.round(s,3))
print(r.go(s, seconds=8))
p,qq = r.fk_world(); import scipy.spatial.transform as st
R = st.Rotation.from_quat(qq).as_matrix(); print('hand', p.round(3), 'tcp', (p+rob.TCP*R[:,2]).round(3)); print('R\n', R.round(2))
"

# openrua op 54
timeout 900 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
Rg = rob.R_grasp(); qg = rob.q_of(Rg)
q0,_ = r.joints()
for z in (1.06, 1.02):
    s = r.ik_best([-0.147,0.075,z], qg, seed=q0, tries=8)
    print('z',z,'q', np.round(s,3), 'maxdiff', np.abs(np.array(s)-np.array(q0)).max().round(3))
    print(r.go(s, seconds=4))
    q0 = s
p,qq = r.fk_world(); R = Rot.from_quat(qq).as_matrix(); print('tcp', (p+rob.TCP*R[:,2]).round(4))
print('close ->', r.gripper(0.0))
import time
for i in range(3):
    print('gap', round(r.finger_gap(),4))
s = r.ik_best([-0.147,0.075,1.25], qg, seed=q0, tries=8)
print('lift', r.go(s, seconds=4))
print('gap after lift', round(r.finger_gap(),4))
" && python3 tools/perception/cam_snap.py frontview frontview.png

# openrua op 55
timeout 900 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
D = np.array(rob.D); k = np.cross([0,0,-1.0], D); k/=np.linalg.norm(k)
Rg = rob.R_grasp()
def R_tilt(th, spin): return Rot.from_rotvec(k*np.radians(th)).as_matrix() @ Rg @ Rot.from_euler('z', spin, degrees=True).as_matrix()
q0,_ = r.joints(); print('start', np.round(q0,3))
path=[]
# swing in place
for f in (0.2,0.4,0.6,0.8,1.0):
    path.append(([-0.147,0.075,1.25], R_tilt(55*f, 180*f)))
Rf = R_tilt(55,180)
final_tcp = np.array([0.06,0.19,0.955]) - 0.115*Rf[:,2]
for zz in (1.25, 1.15, final_tcp[2]):
    path.append(([final_tcp[0], final_tcp[1], zz], Rf))
qs=[]; qprev=q0
for i,(p,R) in enumerate(path):
    s = r.ik_best(p, rob.q_of(R), seed=qprev, tries=12)
    if s is None: print(i,'IK FAIL'); break
    d = np.abs(np.array(s)-np.array(qprev)).max()
    L = r.fk_links(s)
    print(i, np.round(p,3), 'q', np.round(s,2), 'maxdiff', d.round(2), 'l7 z', L['panda_link7'][2].round(3), 'l5', L['panda_link5'].round(2))
    qs.append(s); qprev=s
np.save('path_qs.npy', np.array(qs))
"

# openrua op 56
timeout 1500 python3 -u -c "
import rob, numpy as np
r = rob.init()
qs = np.load('path_qs.npy')
for i,s in enumerate(qs):
    ok = r.go(list(s), seconds=4)
    print(i, ok, 'gap', round(r.finger_gap(),4))
    if r.finger_gap() < 0.008: print('LOST BOTTLE?'); break
p,qq = r.fk_world(); print('hand', p.round(3))
" && python3 tools/perception/cam_snap.py frontview frontview.png && python3 tools/perception/cam_snap.py agentview agentview.png

# openrua op 57
timeout 900 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
print('open ->', r.gripper(0.04))
print('gap', round(r.finger_gap(),4))
# retreat straight up along -hand z direction (back along the approach) then up
q0,_ = r.joints(); p,qq = r.fk_world(); R = Rot.from_quat(qq).as_matrix()
tcp = p + rob.TCP*R[:,2]
tgt = tcp - 0.06*R[:,2] + np.array([0,0,0.08])
s = r.ik_best(tgt, qq, seed=q0, tries=8); print('retreat1', r.go(s, seconds=4))
tgt2 = tgt + np.array([-0.05,-0.05,0.12])
s = r.ik_best(tgt2, qq, seed=s, tries=8); print('retreat2', r.go(s, seconds=4))
print('hand', r.fk_world()[0].round(3))
" && python3 tools/perception/cam_snap.py frontview frontview.png && python3 tools/perception/cam_snap.py agentview agentview.png && timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py sideview

# openrua op 58
python3 -c "
import numpy as np
for cam in ['birdview','sideview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    # points inside tray footprint above tray floor
    t=P[(P[:,0]>-0.085)&(P[:,0]<0.115)&(P[:,1]>0.09)&(P[:,1]<0.226)&(P[:,2]>0.93)&(P[:,2]<1.0)]
    print(cam,'pts above tray floor inside footprint',len(t))
    if len(t): print('  x',t[:,0].min().round(3),t[:,0].max().round(3),'y',t[:,1].min().round(3),t[:,1].max().round(3),'z',t[:,2].min().round(3),t[:,2].max().round(3))
    # anything bottle-like outside the tray in the region near drawer (z 0.91..1.0, y -0.15..0.09)
    o=P[(P[:,0]>-0.2)&(P[:,0]<0.2)&(P[:,1]>-0.15)&(P[:,1]<0.06)&(P[:,2]>0.915)&(P[:,2]<0.975)]
    print('  pts in front of drawer (handle region etc.)',len(o))
    # drawer front face y position: points with x in 0.0..0.1, z 0.93..0.98 , y<0.09
    f=P[(P[:,0]>-0.08)&(P[:,0]<0.11)&(P[:,2]>0.93)&(P[:,2]<0.98)&(P[:,1]<0.09)&(P[:,1]>-0.15)]
    if len(f): print('  drawer front-ish y range',f[:,1].min().round(3),f[:,1].max().round(3))
"


# openrua op 59
python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
t=P[(P[:,0]>-0.078)&(P[:,0]<0.108)&(P[:,1]>0.097)&(P[:,1]<0.226)&(P[:,2]>0.935)&(P[:,2]<1.0)]
print(len(t),'x',t[:,0].min().round(3),t[:,0].max().round(3),'y',t[:,1].min().round(3),t[:,1].max().round(3),'z',t[:,2].min().round(3),t[:,2].max().round(3))
print('centroid',t.mean(0).round(3))
# floor points visible?
fl=P[(P[:,0]>-0.078)&(P[:,0]<0.108)&(P[:,1]>0.097)&(P[:,1]<0.226)&(P[:,2]>0.915)&(P[:,2]<0.935)]
print('floor pts',len(fl))
"


# openrua op 60
python3 -c "
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
h=P[(P[:,1]>-0.12)&(P[:,1]<0.06)&(P[:,2]>0.93)&(P[:,0]>-0.12)&(P[:,0]<0.14)]
print(len(h))
for y0 in np.arange(-0.12,0.06,0.01):
    s=h[(h[:,1]>=y0)&(h[:,1]<y0+0.01)]
    if len(s): print(f'y {y0:+.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"


# openrua op 61
timeout 900 python3 -u -c "
import rob, numpy as np
r = rob.init()
s = r.ik_natural([-0.30,-0.20,1.30], rob.q_of(rob.R_grasp()))
print(np.round(s,2)); print(r.go(s, seconds=6))
" && timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py sideview && timeout 120 python3 cloud.py frontview && python3 -c "
import numpy as np
for cam in ['birdview','sideview','frontview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    h=P[(P[:,1]>-0.12)&(P[:,1]<0.10)&(P[:,2]>0.91)&(P[:,0]>-0.14)&(P[:,0]<0.16)]
    print(cam,len(h))
    for y0 in np.arange(-0.12,0.10,0.01):
        s=h[(h[:,1]>=y0)&(h[:,1]<y0+0.01)]
        if len(s): print(f'  y {y0:+.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"

# openrua op 62
timeout 1500 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
print('close fingers ->', r.gripper(0.0), 'gap', round(r.finger_gap(),4))
t = np.radians(35)
z = np.array([0, np.sin(t), -np.cos(t)]); y = np.array([1.0,0,0]); x = np.cross(y,z)
R = np.column_stack([x,y,z]); print('R\n', R.round(3)); qq = rob.q_of(R)
X = -0.085
s = r.ik_natural([X, 0.03, 1.08], qq, verbose=True)
print('go start', r.go(s, seconds=6))
s = r.ik_best([X, 0.035, 0.955], qq, seed=s, tries=8); print('descend', r.go(s, seconds=4))
p,q_ = r.fk_world(); Rr = Rot.from_quat(q_).as_matrix(); print('tcp', (p+rob.TCP*Rr[:,2]).round(3))
for yy in (0.08, 0.125, 0.17, 0.215):
    s2 = r.ik_best([X, yy, 0.955], qq, seed=s, tries=8)
    res = r.go(s2, seconds=4, attempts=2); s = s2
    p,q_ = r.fk_world(); Rr = Rot.from_quat(q_).as_matrix()
    print('push to', yy, res, 'tcp', (p+rob.TCP*Rr[:,2]).round(3))
" && python3 tools/perception/cam_snap.py frontview frontview.png

# openrua op 63
timeout 900 python3 -u -c "
import rob, numpy as np
from scipy.spatial.transform import Rotation as Rot
r = rob.init()
q0,_ = r.joints(); p,qq = r.fk_world(); R = Rot.from_quat(qq).as_matrix()
tcp = p + rob.TCP*R[:,2]
s = r.ik_best(tcp + np.array([0,-0.04,0.10]), qq, seed=q0); print('back-up', r.go(s, seconds=4))
s = r.ik_natural([-0.30,-0.20,1.30], rob.q_of(rob.R_grasp())); print('park', r.go(s, seconds=6))
print('open ->', r.gripper(0.04))
" && timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py frontview && timeout 120 python3 cloud.py agentview && python3 tools/perception/cam_snap.py agentview agentview.png && python3 -c "
import numpy as np
for cam in ['birdview','frontview','agentview']:
    P=np.load(f'{cam}_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
    # drawer/cabinet front region: x -0.12..0.15, y -0.1..0.30, z 0.91..1.13 -- profile by y
    h=P[(P[:,0]>-0.11)&(P[:,0]<0.14)&(P[:,2]>0.915)&(P[:,2]<1.13)&(P[:,1]>0.0)&(P[:,1]<0.30)]
    print(cam, 'front-region pts', len(h))
    for y0 in np.arange(0.0,0.30,0.01):
        s=h[(h[:,1]>=y0)&(h[:,1]<y0+0.01)]
        if len(s): print(f'  y {y0:+.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
    # any bottle-like points left on the table outside cabinet (dark green bottle ~ z 0.91-1.06)
    o=P[(P[:,2]>0.915)&(P[:,2]<1.08)&(P[:,1]<0.20)&(P[:,1]>-0.15)&(P[:,0]>-0.20)&(P[:,0]<0.2)]
    print('  loose pts near workspace (bowl expected ~600):', len(o), 'x' , o[:,0].min().round(2) if len(o) else '', o[:,0].max().round(2) if len(o) else '', 'y', o[:,1].min().round(2) if len(o) else '', o[:,1].max().round(2) if len(o) else '')
"

# openrua op 64
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Verified quirks of the /workspace Panda ROS 2 simulator (IK frame/tip-link, gripper, TF QoS, unsettled scene) learned while doing the bottle-in-drawer task
metadata:
  type: project
---

Verified facts about the Panda simulator in /workspace (Sept 2026, bottle-in-drawer task, completed successfully):
- /compute_ik and /compute_fk answer in the WORLD frame (not panda_link0), and the IK tip link is panda_link8 = panda_hand rotated 45° about z → multiply the desired hand quaternion by Rz(+45°) before calling IK. IK ignores collisions and returns contorted branches; score/seed solutions (see /workspace/rob.py `ik_natural`, `ik_best`).
- Large joint moves return FollowJointTrajectory error -5 (goal tolerance) from controller lag; re-sending the same goal converges (`Robot.go` loop).
- Gripper only stops at open/closed; result returns before fingers finish — re-read finger gap after.
- Camera TFs are on /tf (not /tf_static); /tf_static needs TRANSIENT_LOCAL + RELIABLE QoS. Wrist camera clips below ~0.3 m.
- Scene is unsettled at start (objects float several cm) — re-measure after the first motion before trusting heights.
- Clock is paused between commands; verify via fresh sensor reads.

**Why:** these cost most of the debugging time in the first session.
**How to apply:** reuse /workspace/rob.py and /workspace/cloud.py instead of rederiving; see [[panda-drawer-scene-geometry]] if it exists.
OPENRUA_EOF

# openrua op 65
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda sim quirks](panda-sim-quirks.md) — IK world frame + link8 45° tip, -5 retry, gripper open/closed only, TF QoS, unsettled scene" >> MEMORY.md && cat MEMORY.md
