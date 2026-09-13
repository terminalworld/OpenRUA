#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 3
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
ls -la .ros/log; tail -n 30 .ros/log/*.log | head -150; timeout 20 ros2 service list

# openrua op 5
timeout 60 python3 - <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("tfdump")
got = {}
def cb(m, which):
    for t in m.transforms:
        got[(which, t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage, "/tf_static", lambda m: cb(m,"static"), qos)
n.create_subscription(TFMessage, "/tf", lambda m: cb(m,"dyn"), 10)
t0=time.time()
while time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
for k,v in sorted(got.items()):
    tr, r = v.translation, v.rotation
    print(f"{k[0]:6s} {k[1]:>28s} -> {k[2]:<28s} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({r.x:.4f},{r.y:.4f},{r.z:.4f},{r.w:.4f})")
EOF

# openrua op 6
timeout 120 python3 - <<'EOF'
import rclpy, numpy as np, time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
rclpy.init(); n = rclpy.create_node("pc")
def grab(topic, T):
    got={}
    s=n.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_subscription(s); return got["m"]
for cam in ["birdview","agentview"]:
    info = grab(f"/{cam}/color/camera_info", CameraInfo)
    d = CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw", Image), "passthrough")
    print(cam, d.shape, d.dtype, "K=", np.array(info.k).reshape(3,3).tolist(), "depth range", np.nanmin(d), np.nanmax(d))
    np.save(f"{cam}_depth.npy", d)
EOF

# openrua op 7
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
d = np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# birdview optical frame at world (-0.2,0,3.0), q=(0.7071,0.7071,0,0)
qx,qy,qz,qw = 0.7071,0.7071,0,0
x,y,z,w=qx,qy,qz,qw
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([-0.2,0,3.0])
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
pc = np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
world = pc@R.T + t
np.save("birdview_world.npy", world)
# table height: mode of z
zs = world[...,2]
hist,edges=np.histogram(zs[np.isfinite(zs)],bins=200)
i=np.argmax(hist); table_z=(edges[i]+edges[i+1])/2
print("table z ~", table_z)
# objects above table
mask = (zs>table_z+0.01)&(zs<table_z+0.5)
mask[:, :] &= (world[...,0] > -0.55)  # exclude robot base region? robot is at x=-0.75
mask8=(mask*255).astype(np.uint8)
nlab,lab,stats,cent=cv2.connectedComponentsWithStats(mask8)
for i in range(1,nlab):
    if stats[i,4]<30: continue
    m=lab==i
    pts=world[m]
    print(f"blob {i}: px area {stats[i,4]} bbox(u,v,w,h)={stats[i,:4].tolist()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
EOF

# openrua op 8
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
world=np.load("birdview_world.npy"); zs=world[...,2]
# crop caddy region + book: u 200..350, v 140..300
crop = zs[140:300, 200:360]
print("z percentiles table region:", np.percentile(zs[300:340, 300:400], [1,50,99]))
# heightmap in cm above table, clip 0..15
h = np.clip((crop-0.884)*100, 0, 20)
img = (h/20*255).astype(np.uint8)
img = cv2.resize(img, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
cv2.imwrite("caddy_height.png", img)
# print stats of heights for caddy region (exclude arm z>1.05)
for name,(v0,v1,u0,u1) in {"caddy":(155,225,215,310)}.items():
    c=zs[v0:v1,u0:u1]
    print(name, np.unique(np.round(c[c<1.05],2), return_counts=True))
EOF

# openrua op 9
timeout 30 ros2 param list /robot_bridge; timeout 30 ros2 param dump /robot_bridge 2>/dev/null | head -50; timeout 20 ros2 node info /robot_bridge

# openrua op 10
for c in galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 11
timeout 60 python3 - <<'EOF'
import numpy as np, cv2
world=np.load("birdview_world.npy"); zs=world[...,2]
table=0.8804
# Book: blob near u 303-331, v 253-285
m=np.zeros(zs.shape,bool); m[245:295,295:340]=True
m &= (zs>table+0.02)&(zs<1.2)
pts=world[m]
print("book pts", len(pts), "z range", pts[:,2].min(), pts[:,2].max())
top = pts[pts[:,2]>1.06]
print("book top pts", len(top), "center", top[:,:2].mean(0))
xy=top[:,:2]-top[:,:2].mean(0)
w,v=np.linalg.eigh(xy.T@xy)
d=v[:,1]; n=v[:,0]
print("long axis dir", d, "yaw deg", np.degrees(np.arctan2(d[1],d[0])), "extent along d", (xy@d).min(), (xy@d).max(), "along n", (xy@n).min(), (xy@n).max())
# Caddy: region u 215-310, v 150-230, use heights between table+0.02 and 1.05 (walls), exclude arm
c=np.zeros(zs.shape,bool); c[150:235,210:320]=True
walls = c & (zs>table+0.05) & (zs<1.05)
wp=world[walls]
print("caddy wall pts", len(wp), "x", wp[:,0].min(), wp[:,0].max(), "y", wp[:,1].min(), wp[:,1].max(), "z pct", np.percentile(wp[:,2],[10,50,90,99]))
# find wall lines: histogram of x and y for wall points
hx,ex=np.histogram(wp[:,0],bins=np.arange(-0.5,-0.2,0.005)); 
hy,ey=np.histogram(wp[:,1],bins=np.arange(-0.4,0.15,0.005))
print("x hist:"); [print(f"  {ex[i]:.3f} {hx[i]}") for i in range(len(hx)) if hx[i]>20]
print("y hist:"); [print(f"  {ey[i]:.3f} {hy[i]}") for i in range(len(hy)) if hy[i]>20]
EOF

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: FK/IK, trajectory, gripper, joint state, in one node.

Coordinates for ik()/fk() are in panda_link0 (planning frame); use w2b()/b2w()
to convert to/from world (panda_link0 sits at world (-0.75, 0, 0.912)).
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_W = np.array([-0.75, 0.0, 0.912])
TCP = M["hand"]["tcp_offset_m"]


def w2b(p):
    return np.asarray(p, float) - BASE_W


def b2w(p):
    return np.asarray(p, float) + BASE_W


def quat_topdown(yaw_x):
    """Hand z down; hand x-axis at angle yaw_x (rad) in the world xy plane.
    Fingers close along hand y = (sin a, -cos a, 0)."""
    a = yaw_x
    R = np.array([[np.cos(a), np.sin(a), 0.0],
                  [np.sin(a), -np.cos(a), 0.0],
                  [0.0, 0.0, -1.0]]).T  # columns = hand axes
    return R_to_quat(R)


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x y z w


def quat_to_R(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def _seed(self, arm=None):
        js = JointState()
        arm = arm if arm is not None else self.arm()
        js.name = list(JOINTS)
        js.position = [float(v) for v in arm]
        return js

    def fk(self, arm=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(arm)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def ik(self, pos_b, quat, seed=None, at_tcp=False, timeout=30):
        """pos_b in panda_link0 frame; if at_tcp, pos is the TCP target."""
        pos_b = np.asarray(pos_b, float)
        if at_tcp:
            R = quat_to_R(quat)
            pos_b = pos_b - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK timeout")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, positions, seconds=3.0, via=None):
        """Send one trajectory; via = optional list of (positions, t)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for q, t in (via or []) + [(positions, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = self.arm()
        err = np.abs(np.array(now) - np.array(positions)).max()
        print(f"move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def grab_img(self, topic, timeout=30):
        got = {}
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get("m")

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self.grab_img(f"/{cam}/color/image_raw")
        img = CvBridge().imgmsg_to_cv2(m, "bgr8")
        cv2.imwrite(out or f"/workspace/{cam}.png", img)
        return img

    def depth(self, cam):
        from cv_bridge import CvBridge
        m = self.grab_img(f"/{cam}/depth/image_raw")
        return CvBridge().imgmsg_to_cv2(m, "passthrough")
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
print("arm", np.round(r.arm(),4), "fingers", r.fingers())
p,q = r.fk()
print("hand base-frame", p, "world", b2w(p), "quat", q)
print("R cols", np.round(quat_to_R(q),3))
# test IK for a top-down pose above the book
book = np.array([-0.097,-0.016,1.075])
yaw = np.radians(137.5)   # hand x along the book long axis
qd = quat_topdown(yaw)
print("qd", qd, "R", np.round(quat_to_R(qd),3))
for z in [1.25, 1.20]:
    sol = r.ik(w2b(book+[0,0,z-1.075]), qd, at_tcp=True)
    print("IK z", z, sol and np.round(sol,3))
EOF

# openrua op 14
timeout 60 python3 - <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("rd")
got={}
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m",m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time; t=time.time()
while "m" not in got and time.time()-t<10: rclpy.spin_once(n, timeout_sec=0.2)
s=got["m"].data
open("/workspace/robot.urdf","w").write(s)
import re
for m in re.finditer(r'<joint name="([^"]+)" type="fixed">.*?<parent link="([^"]+)"/>.*?<child link="([^"]+)"/>.*?<origin([^/]*)/>', s, re.S):
    print(m.group(1), m.group(2), "->", m.group(3), m.group(4).strip())
print([l for l in re.findall(r'<link name="([^"]+)"', s)])
EOF

# openrua op 15
grep -n -A6 'name="panda_joint8"\|name="panda_hand_joint"\|name="panda_finger_joint1"' /workspace/robot.urdf | head -60; grep -n "world\|virtual" /workspace/robot.urdf | head

# openrua op 16
timeout 60 python3 - <<'EOF'
import numpy as np
from scipy.spatial.transform import Rotation as Rot
# panda FK from URDF joint origins
q=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
def T(rpy,xyz,theta):
    M=np.eye(4); M[:3,:3]=Rot.from_euler('xyz',rpy).as_matrix(); M[:3,3]=xyz
    Rz=np.eye(4); Rz[:3,:3]=Rot.from_euler('z',theta).as_matrix()
    return M@Rz
J=[((0,0,0),(0,0,0.333)),((-np.pi/2,0,0),(0,0,0)),((np.pi/2,0,0),(0,-0.316,0)),((np.pi/2,0,0),(0.0825,0,0)),((-np.pi/2,0,0),(-0.0825,0.384,0)),((np.pi/2,0,0),(0,0,0)),((np.pi/2,0,0),(0.088,0,0))]
M=np.eye(4)
for (rpy,xyz),th in zip(J,q): M=M@T(rpy,xyz,th)
M=M@T((0,0,0),(0,0,0.107),0)@T((0,0,-np.pi/4),(0,0,0),0)
print("hand in link0:", M[:3,3], "quat", Rot.from_matrix(M[:3,:3]).as_quat())
print("hand world:", M[:3,3]+np.array([-0.75,0,0.912]))
EOF

# openrua op 17
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
p,q = r.fk()
sol = r.ik(p, q)
print("IK current pose (world coords):", sol and np.round(sol,3))
book = np.array([-0.097,-0.016,1.075])
for yaw in [137.5, 137.5-180]:
    qd = quat_topdown(np.radians(yaw))
    for z in [1.30, 1.25, 1.20]:
        sol = r.ik(book+[0,0,z-1.075], qd, at_tcp=True)
        print("yaw",yaw,"IK tcp z", z, sol and np.round(sol,3))
EOF

# openrua op 18
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
r.gripper(0.04)
book = np.array([-0.097,-0.016,1.075])
qd = quat_topdown(np.radians(-42.5))
pre = r.ik(book+[0,0,0.10], qd, at_tcp=True)
print("pre", np.round(pre,3))
r.move(pre, 4.0)
p,q = r.fk()
print("hand world", p, "tcp", p + TCP*quat_to_R(q)[:,2])
print("fingers", r.fingers())
r.snap("birdview"); r.snap("robot0_eye_in_hand"); r.snap("agentview")
EOF

# openrua op 19
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
d = r.depth("birdview")
fx=fy=579.4112549695428; cx=320; cy=240
R = np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@R.T+t
np.save("birdview_world.npy", world)
zs=world[...,2]
print("table z", np.percentile(zs[300:340,300:400],50))
# book region: around world x -0.1, y -0.016 -> pixel: u = 320 + (y-0)/... compute mask by world coords
m=(np.abs(world[...,0]+0.097)<0.09)&(np.abs(world[...,1]+0.016)<0.09)&(zs>0.90)&(zs<1.12)
pts=world[m]
print("pts", len(pts), "z range", pts[:,2].min(), pts[:,2].max())
top=pts[pts[:,2]>pts[:,2].max()-0.015]
print("top center", top[:,:2].mean(0), "n", len(top))
xy=top[:,:2]-top[:,:2].mean(0); w,v=np.linalg.eigh(xy.T@xy); dd=v[:,1]
print("yaw", np.degrees(np.arctan2(dd[1],dd[0])), "len", (xy@dd).max()-(xy@dd).min())
# mug
m2=(np.abs(world[...,0]+0.18)<0.08)&(np.abs(world[...,1]-0.16)<0.08)&(zs>0.90)&(zs<1.2)
p2=world[m2]; print("mug z max", p2[:,2].max(), "center", p2[:,:2].mean(0))
EOF

# openrua op 20
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
d = r.depth("agentview")
fx=fy=579.4112549695428; cx=320; cy=240
R = quat_to_R([0.6380,0.6380,-0.3048,-0.3048]); t=np.array([0.4586,0.0,1.6104])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@R.T+t
zs=world[...,2]
print("table z sample", np.percentile(zs[420:470,100:600],[5,50,95]))
m=(np.abs(world[...,0]+0.10)<0.10)&(np.abs(world[...,1]+0.016)<0.10)&(zs>0.90)&(zs<1.12)
pts=world[m]
print("book pts", len(pts), "x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
top=pts[pts[:,2]>pts[:,2].max()-0.01]
print("top center", top[:,:2].mean(0), "n", len(top))
xy=top[:,:2]-top[:,:2].mean(0); w,v=np.linalg.eigh(xy.T@xy); dd=v[:,1]
print("yaw", np.degrees(np.arctan2(dd[1],dd[0])), "len", (xy@dd).max()-(xy@dd).min(), "thick", (xy@v[:,0]).max()-(xy@v[:,0]).min())
# pixel coords of top
us=uu[m][pts[:,2]>pts[:,2].max()-0.01]; vs=vv[m][pts[:,2]>pts[:,2].max()-0.01]
print("top pixels u", us.min(), us.max(), "v", vs.min(), vs.max())
EOF

# openrua op 21
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
top = np.array([-0.0956,-0.0135,1.022])
qd = quat_topdown(np.radians(-41.7))
mid = r.ik(top+[0,0,0.06], qd, at_tcp=True)
grasp = r.ik(top+[0,0,-0.04], qd, at_tcp=True)
print("mid", np.round(mid,3)); print("grasp", np.round(grasp,3))
r.move(grasp, 3.0, via=[(mid, 1.5)])
p,q = r.fk(); print("tcp", p + TCP*quat_to_R(q)[:,2])
print("wrench", np.round(r.wrench(),2))
r.snap("robot0_eye_in_hand"); r.snap("agentview")
EOF

# openrua op 22
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
f = r.gripper(0.0)
for i in range(3):
    r.move(r.arm(), 0.3)  # tiny no-op trajectory to advance sim clock
    print("fingers", r.fingers(), "wrench", np.round(r.wrench(),2))
EOF

# openrua op 23
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
for i in range(6):
    r.move(r.arm(), 0.5)
    f=r.fingers(); print("fingers", f, "gap", f[0]-f[1], "wrench", np.round(r.wrench(),2))
r.snap("agentview")
EOF

# openrua op 24
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
for i in range(4):
    r.move(r.arm(), 0.5)
    f=r.fingers(); print("fingers", f, "gap", f[0]-f[1], "wrench", np.round(r.wrench(),2))
r.snap("agentview")
EOF

# openrua op 25
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
p,q = r.fk()
lift = r.ik(p+[0,0,0.30], q)
print("lift", np.round(lift,3))
r.move(lift, 3.0)
p,q = r.fk(); print("tcp", p + TCP*quat_to_R(q)[:,2])
f=r.fingers(); print("fingers", f, "gap", f[0]-f[1], "wrench", np.round(r.wrench(),2))
r.snap("agentview"); r.snap("birdview")
EOF

# openrua op 26
timeout 120 python3 -u - <<'EOF'
from rob import *
r = Robot()
print("arm", np.round(r.arm(),3))
p,q=r.fk(); print("hand", p, "R", np.round(quat_to_R(q),3))
d = r.depth("birdview")
fx=fy=579.4112549695428; cx=320; cy=240
R = np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@R.T+t
zs=world[...,2]
walls=(world[...,0]>-0.5)&(world[...,0]<-0.30)&(world[...,1]>-0.4)&(world[...,1]<0.1)&(zs>0.93)&(zs<1.06)
wp=world[walls]
print("wall pts", len(wp), "z pct", np.percentile(wp[:,2],[50,90,99]))
hx,ex=np.histogram(wp[:,0],bins=np.arange(-0.5,-0.3,0.005)); hy,ey=np.histogram(wp[:,1],bins=np.arange(-0.4,0.1,0.005))
print("x:", [(round(ex[i],3),int(hx[i])) for i in range(len(hx)) if hx[i]>15])
print("y:", [(round(ey[i],3),int(hy[i])) for i in range(len(hy)) if hy[i]>15])
import cv2
crop=np.clip((zs[130:240,180:340]-0.88)*100,0,25); img=cv2.applyColorMap(cv2.resize((crop/25*255).astype(np.uint8),None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST),cv2.COLORMAP_JET); cv2.imwrite("caddy_height2.png",img)
EOF

# openrua op 27
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = r.arm(); print("arm", np.round(arm,3))
p,q=r.fk(); R=quat_to_R(q); print("hand", p, "hand x", R[:,0], "yaw", np.degrees(np.arctan2(R[1,0],R[0,0])))
tcp = p + TCP*R[:,2]; print("tcp", tcp)
target = np.array([-0.435, -0.133, 1.165])
qd = quat_topdown(np.radians(90))
above = r.ik([target[0], target[1], tcp[2]], qd, at_tcp=True, seed=arm)
print("above", above and np.round(above,3), "delta", above and np.round(np.array(above)-arm,2))
place = above and r.ik(target, qd, at_tcp=True, seed=above)
print("place", place and np.round(place,3), "delta", place and np.round(np.array(place)-np.array(above),2))
EOF

# openrua op 28
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = np.array(r.arm())
target = np.array([-0.435, -0.133, 1.165])
for a in [90, -90]:
    qd = quat_topdown(np.radians(a))
    for tz in [1.283, 1.22]:
        for seedname, seed in [("cur", arm), ("nom", [-0.4, -0.3, 0.0, -2.3, 0.0, 2.0, 0.4 if a==-90 else 1.5]), ("nom2", [-0.4, 0.2, 0.0, -1.8, 0.0, 2.0, 0.4 if a==-90 else 1.5])]:
            s = r.ik([target[0], target[1], tz], qd, at_tcp=True, seed=seed)
            print(f"a={a} tz={tz} seed={seedname}:", s and np.round(s,2), s and f"maxdelta_cur={np.abs(np.array(s)-arm).max():.2f}")
EOF

# openrua op 29
timeout 900 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = np.array(r.arm())
target = np.array([-0.435, -0.133, 1.165])
qd = quat_topdown(np.radians(-90))
mid = r.ik([-0.26, -0.07, 1.33], qd, at_tcp=True, seed=arm)
above = r.ik([target[0], target[1], 1.283], qd, at_tcp=True, seed=mid)
print("mid", np.round(mid,3)); print("above", np.round(above,3))
code, err = r.move(above, 6.0, via=[(mid, 3.0)])
if err > 0.05:
    print("resending"); r.move(above, 3.0)
p,q=r.fk(); R=quat_to_R(q); print("tcp", p+TCP*R[:,2], "hand x", np.round(R[:,0],3))
f=r.fingers(); print("fingers", f, "gap", f[0]-f[1])
r.snap("agentview"); r.snap("birdview"); r.snap("frontview")
EOF

# openrua op 30
timeout 600 python3 -u - <<'EOF'
from rob import *
from scipy.spatial.transform import Rotation as Rot
r = Robot()
arm = np.array(r.arm())
p,q=r.fk(); print("current q", q, "R", np.round(quat_to_R(q),3))
for a in [-90, -45, 0, 45]:
    qd = quat_topdown(np.radians(a))
    sol = r.ik([-0.435,-0.133,1.283], qd, at_tcp=True, seed=arm)
    if sol is None: print(a, "no sol"); continue
    p2,q2 = r.fk(sol)
    R2=quat_to_R(q2)
    ang = np.degrees((Rot.from_quat(q2)*Rot.from_quat(qd).inv()).magnitude())
    print(f"a={a}: sol {np.round(sol,2)} fk hand x yaw {np.degrees(np.arctan2(R2[1,0],R2[0,0])):.1f} hand z {np.round(R2[:,2],3)} orient err {ang:.1f} deg, tcp {np.round(p2+TCP*R2[:,2],3)}")
EOF

# openrua op 31
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = np.array(r.arm())
qd = quat_topdown(np.radians(-135))
sol = r.ik([-0.435,-0.133,1.283], qd, at_tcp=True, seed=arm)
print("sol", np.round(sol,3), "delta", np.round(np.array(sol)-arm,2))
p2,q2=r.fk(sol); R2=quat_to_R(q2); print("predicted hand x yaw", np.degrees(np.arctan2(R2[1,0],R2[0,0])))
r.move(sol, 3.0)
p,q=r.fk(); R=quat_to_R(q); print("tcp", p+TCP*R[:,2], "hand x yaw", np.degrees(np.arctan2(R[1,0],R[0,0])))
f=r.fingers(); print("fingers", f, "gap", f[0]-f[1])
r.snap("agentview"); r.snap("birdview")
# locate book from agentview depth: points hanging under the hand
d = r.depth("agentview")
fx=fy=579.4112549695428; cx=320; cy=240
Ra = quat_to_R([0.6380,0.6380,-0.3048,-0.3048]); t=np.array([0.4586,0.0,1.6104])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@Ra.T+t
zs=world[...,2]
tcp=p+TCP*R[:,2]
m=(np.abs(world[...,0]-tcp[0])<0.12)&(np.abs(world[...,1]-tcp[1])<0.12)&(zs>tcp[2]-0.105)&(zs<tcp[2]-0.005)
pts=world[m]; print("book pts", len(pts))
if len(pts):
    print("x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
    # the camera sees the +x face; estimate y-center from y extent
    print("y center", (pts[:,1].min()+pts[:,1].max())/2, "tcp", tcp)
EOF

# openrua op 32
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = np.array(r.arm())
sol = [-0.432,-0.685,0.028,-2.591,0.019,1.906,1.939]
r.move(sol, 2.0)
print("arm", np.round(r.arm(),3))
p,q=r.fk(); R=quat_to_R(q); tcp=p+TCP*R[:,2]; print("tcp", tcp, "hand x yaw", np.degrees(np.arctan2(R[1,0],R[0,0])))
d = r.depth("agentview")
fx=fy=579.4112549695428; cx=320; cy=240
Ra = quat_to_R([0.6380,0.6380,-0.3048,-0.3048]); t=np.array([0.4586,0.0,1.6104])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@Ra.T+t
zs=world[...,2]
m=(np.abs(world[...,0]-tcp[0])<0.12)&(np.abs(world[...,1]-tcp[1])<0.12)&(zs>tcp[2]-0.105)&(zs<tcp[2]-0.005)
pts=world[m]; print("book pts", len(pts))
print("x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
# fit face plane orientation: PCA on xy of face points
xy=pts[:,:2]-pts[:,:2].mean(0); w,v=np.linalg.eigh(xy.T@xy); dd=v[:,1]
print("book face dir yaw", np.degrees(np.arctan2(dd[1],dd[0])), "y center", (pts[:,1].min()+pts[:,1].max())/2)
EOF

# openrua op 33
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm = np.array(r.arm())
qd = quat_topdown(np.radians(-135))
sol = r.ik([-0.435,-0.137,1.165], qd, at_tcp=True, seed=arm)
print("sol", np.round(sol,3), "delta", np.round(np.array(sol)-arm,2))
code, err = r.move(sol, 3.0)
if err > 0.02: r.move(sol, 2.0)
p,q=r.fk(); R=quat_to_R(q); tcp=p+TCP*R[:,2]; print("tcp", tcp, "hand x yaw", np.degrees(np.arctan2(R[1,0],R[0,0])))
f=r.fingers(); print("fingers", f, "gap", f[0]-f[1])
d = r.depth("agentview")
fx=fy=579.4112549695428; cx=320; cy=240
Ra = quat_to_R([0.6380,0.6380,-0.3048,-0.3048]); t=np.array([0.4586,0.0,1.6104])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@Ra.T+t
zs=world[...,2]
m=(np.abs(world[...,0]-tcp[0])<0.12)&(np.abs(world[...,1]-tcp[1])<0.12)&(zs>tcp[2]-0.105)&(zs<tcp[2]-0.005)
pts=world[m]; print("book pts", len(pts))
print("x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max(), "y center", (pts[:,1].min()+pts[:,1].max())/2)
r.snap("agentview"); r.snap("sideview"); r.snap("frontview")
EOF

# openrua op 34
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
r.gripper(0.04)
for i in range(4):
    r.move(r.arm(), 0.5)
print("fingers", r.fingers())
r.snap("agentview"); r.snap("sideview"); r.snap("frontview")
EOF

# openrua op 35
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
arm=np.array(r.arm())
qd = quat_topdown(np.radians(-135))
up = r.ik([-0.25,-0.05,1.40], qd, at_tcp=True, seed=arm)
print("up", np.round(up,3))
r.move(up, 3.0)
r.snap("birdview"); r.snap("agentview"); r.snap("frontview")
d = r.depth("birdview")
fx=fy=579.4112549695428; cx=320; cy=240
R = np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@R.T+t
zs=world[...,2]
# inside middle column region, anything above table and below wall top
m=(world[...,0]>-0.463)&(world[...,0]<-0.34)&(world[...,1]>-0.195)&(world[...,1]<-0.072)&(zs>0.90)&(zs<1.045)
pts=world[m]; print("pts in middle column above floor", len(pts))
if len(pts):
    print("x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max())
    hi=pts[pts[:,2]>pts[:,2].max()-0.01]; print("top center", hi[:,:2].mean(0), "top z", pts[:,2].max())
# also anything on the table elsewhere (book fell outside?)
m2=(zs>0.90)&(zs<1.05)&(world[...,0]>-0.6)&(world[...,0]<0.3)&(np.abs(world[...,1])<0.5)
import cv2
lab=(m2*255).astype(np.uint8); n,l,s,c=cv2.connectedComponentsWithStats(lab)
for i in range(1,n):
    if s[i,4]<20: continue
    p=world[l==i]; print("blob", s[i,4], "x",round(p[:,0].min(),3),round(p[:,0].max(),3),"y",round(p[:,1].min(),3),round(p[:,1].max(),3),"z",round(p[:,2].min(),3),round(p[:,2].max(),3))
EOF

# openrua op 36
timeout 600 python3 -u - <<'EOF'
from rob import *
r = Robot()
up=[-0.345,-0.188,0.22,-1.864,0.041,1.68,2.223]
r.move(up, 3.0)
print("arm", np.round(r.arm(),3))
r.snap("birdview"); r.snap("agentview"); r.snap("frontview"); r.snap("sideview")
d = r.depth("birdview")
fx=fy=579.4112549695428; cx=320; cy=240
R = np.array([[0,1,0],[1,0,0],[0,0,-1]]); t=np.array([-0.2,0,3.0])
H,W=d.shape; vv,uu=np.mgrid[0:H,0:W]
pc=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1); world=pc@R.T+t
zs=world[...,2]
m=(world[...,0]>-0.46)&(world[...,0]<-0.41)&(world[...,1]>-0.19)&(world[...,1]<-0.075)&(zs>0.92)&(zs<1.045)
pts=world[m]; print("book pts in back compartment", len(pts))
print("x", pts[:,0].min(), pts[:,0].max(), "y", pts[:,1].min(), pts[:,1].max(), "z", pts[:,2].min(), pts[:,2].max(), "center", pts[:,:2].mean(0))
m3=(world[...,0]>-0.40)&(world[...,0]<-0.345)&(world[...,1]>-0.19)&(world[...,1]<-0.075)&(zs>0.92)&(zs<1.045)
print("pts in FRONT compartment interior above floor:", m3.sum())
import cv2
crop=np.clip((zs[130:240,180:340]-0.88)*100,0,25); img=cv2.applyColorMap(cv2.resize((crop/25*255).astype(np.uint8),None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST),cv2.COLORMAP_JET); cv2.imwrite("caddy_height3.png",img)
EOF

# openrua op 37
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-frame-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-frame-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-frame-quirks
description: On the Panda/LIBERO ROS workstation, /compute_fk and /compute_ik work in WORLD coords (not panda_link0), and IK solves for panda_link8 so the resulting panda_hand yaw is +45° from the requested quaternion
metadata:
  type: project
---

On the Franka Panda robot workstation (/workspace, machine.yaml, LIBERO study scene):
- `/compute_fk` (link panda_hand) and `/compute_ik` (group panda_arm, empty frame_id)
  both use WORLD coordinates; panda_link0 sits at world (-0.75, 0, 0.912).
  docs/30-action.md claims the model frame is the arm base — that was wrong here.
- IK targets the group tip (panda_link8): the achieved panda_hand x-axis yaw is
  +45° from the requested quaternion's yaw. Request yaw-45° to get the hand yaw you want,
  and always confirm with FK after the move.
- Long joint7 rotations end with error_code=-5 and ~0.15 rad lag; resending the same
  goal converges (as docs/30-action.md says).
- Scene objects may float at session start; re-measure after the first motion.

**Why:** cost me one bad lift/rotate cycle before diagnosing via IK→FK round trip.
**How to apply:** verify frame/tip conventions with an IK→FK round trip before trusting
any orientation math on this machine.
OPENRUA_EOF

# openrua op 38
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda MoveIt frame quirks](panda-moveit-frame-quirks.md) — FK/IK in world coords; IK yaw is +45° off (link8 tip); joint7 lag → resend" >> MEMORY.md
