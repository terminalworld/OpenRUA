#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; timeout 10 ros2 topic echo /birdview/color/camera_info --once

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        tr=t.transform.translation; q=t.transform.rotation
        seen[(t.header.frame_id, t.child_frame_id)] = (round(tr.x,4),round(tr.y,4),round(tr.z,4),round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()): print(k, v)
EOF
timeout 30 python3 tfdump.py

# openrua op 7
cat > scene.py <<'EOF'
import numpy as np, rclpy, struct
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
rclpy.init(); node=rclpy.create_node("scene")
got={}
def grab(topic, T):
    got.pop('m',None)
    s=node.create_subscription(T, topic, lambda m: got.setdefault('m',m), 1)
    while 'm' not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
d=CvBridge().imgmsg_to_cv2(grab("/birdview/depth/image_raw", Image), "passthrough").astype(float)
info=grab("/birdview/color/camera_info", CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
# birdview optical frame: world (-0.2,0,3.0), quat (0.7071,0.7071,0,0) -> R = rotx(180)? compute
q=np.array([0.7071,0.7071,0,0]); x,y,z,w=q
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t=np.array([-0.2,0,3.0])
H,W=d.shape
vs,us=np.mgrid[0:H,0:W]
Z=d
X=(us-cx)*Z/fx; Y=(vs-cy)*Z/fy
P=np.stack([X,Y,Z],-1)@R.T+t
np.save("bird_world.npy",P)
zw=P[...,2]
print("z range", np.nanmin(zw), np.nanmax(zw))
# histogram of heights
h,e=np.histogram(zw[np.isfinite(zw)], bins=60)
for hh,ee in zip(h,e): 
    if hh>50: print(f"{ee:.3f} {hh}")
EOF
timeout 60 python3 scene.py

# openrua op 8
cat > blobs.py <<'EOF'
import numpy as np, cv2
P=np.load("bird_world.npy"); zw=P[...,2]
img=cv2.imread("birdview.png")
table=zw[(zw>0.85)&(zw<0.9)]
print("table z median", np.median(table))
mask=((zw>0.905)&(zw<1.15)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]
    print(f"blob {i}: px area {stats[i,4]} centroid px {cent[i].round(1)} bbox {stats[i,:4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmed {np.median(pts[:,2]):.3f} color {img[m].mean(0).round()}")
EOF
python3 blobs.py

# openrua op 9
cat > profile.py <<'EOF'
import numpy as np
P=np.load("bird_world.npy")
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.13,-0.30,-0.12),"potB":(-0.13,0.0,0.13,0.31),"stove":(0.06,0.30,-0.08,0.15),"knob":(-0.03,0.09,-0.03,0.09)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.90)
    pts=P[m]
    print(name, len(pts))
    for z in np.arange(0.90,1.07,0.01):
        s=pts[(pts[:,2]>=z)&(pts[:,2]<z+0.01)]
        if len(s)>3: print(f"  z {z:.2f}-{z+0.01:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
EOF
python3 profile.py

# openrua op 10
cat > sideprof.py <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("sp")
got={}
def grab(topic,T):
    got.pop('m',None)
    s=node.create_subscription(T,topic,lambda m: got.setdefault('m',m),1)
    while 'm' not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s); return got['m']
cam="sideview"; t=np.array([-0.0565,1.2761,1.488]); q=(0.0099,0.8064,-0.5912,-0.0069)
d=CvBridge().imgmsg_to_cv2(grab(f"/{cam}/depth/image_raw",Image),"passthrough").astype(float)
info=grab(f"/{cam}/color/camera_info",CameraInfo)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=d.shape; vs,us=np.mgrid[0:H,0:W]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1)@R_of(q).T+t
np.save("side_world.npy",P)
for name,(x0,x1,y0,y1) in {"potB":(-0.13,0.0,0.13,0.31),"potA":(-0.26,-0.13,-0.30,-0.12)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.895)
    pts=P[m]; print(name,len(pts))
    for z in np.arange(0.89,1.07,0.01):
        s=pts[(pts[:,2]>=z)&(pts[:,2]<z+0.01)]
        if len(s)>3: print(f"  z {z:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] w={s[:,0].max()-s[:,0].min():.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF
timeout 60 python3 sideprof.py

# openrua op 11
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -20

# openrua op 12
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small robot helper: joint state, FK, IK, trajectory, gripper, servo.

World frame = panda_link0 + BASE offset (from TF world->panda_link0).
All public poses are in WORLD coordinates; hand pose = panda_hand frame.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R).as_quat()  # x y z w
    return q if q[3] >= 0 else -q


def R_from_axes(hz, hy):
    """Rotation whose columns are hand x,y,z given hand z (approach) and
    hand y (finger-open axis) in world."""
    hz = np.asarray(hz, float); hz /= np.linalg.norm(hz)
    hy = np.asarray(hy, float); hy -= hz * hy.dot(hz); hy /= np.linalg.norm(hy)
    hx = np.cross(hy, hz)
    return np.stack([hx, hy, hz], 1)


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in JOINTS])

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None, link="panda_hand"):
        """World pose (p, R) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(5):
            raise RuntimeError("no FK")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        R = quat_to_R([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def tcp(self, q=None):
        p, R = self.fk(q)
        return p + TCP * R[:, 2], R

    # ---------------- IK ----------------
    def ik(self, p_world, R, seed=None, at_tcp=False, timeout=30):
        p = np.asarray(p_world, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        p = p - BASE
        q = R_to_quat(R)
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(5):
            raise RuntimeError("no IK")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, waypoints=None):
        """Send trajectory to q (optionally via waypoints [(q, t), ...])."""
        if not self.fjt.wait_for_server(10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for wq, wt in (waypoints or []):
            pt = JointTrajectoryPoint(positions=[float(x) for x in wq])
            pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"[move_q] error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, p_world, R, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        return self.move_q(q, seconds)

    def gripper(self, width, timeout=120):
        if not self.grip.wait_for_server(10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"[gripper] reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v_world, n=20, frame=None):
        """Stream n twist messages with linear velocity v (m/s) in base frame."""
        msg = TwistStamped()
        msg.header.frame_id = frame or TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out

    def depth_world(self, cam, t, q):
        """World point cloud (H,W,3) from a camera's depth image."""
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/depth/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        d = CvBridge().imgmsg_to_cv2(got["m"], "passthrough").astype(float)
        got = {}
        sub = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                            lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        k = got["m"].k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        vs, us = np.mgrid[0:H, 0:W]
        P = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
        return P @ quat_to_R(q).T + np.asarray(t)


BIRD = (np.array([-0.2, 0.0, 3.0]), (0.7071, 0.7071, 0.0, 0.0))
OPENRUA_EOF

# openrua op 13
cat > t_fk.py <<'EOF'
from rob import *
r=Rob("t")
q=r.arm_q(); print("q",q.round(3))
p,R=r.fk(q); print("hand world",p.round(4)); print(R.round(3))
pt,_=r.tcp(q); print("tcp world",pt.round(4))
print("fingers", r.fingers())
EOF
timeout 60 python3 t_fk.py

# openrua op 14
cat > t_ik.py <<'EOF'
from rob import *
import rob
r=Rob("t")
q=r.arm_q()
# raw FK
req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand","panda_link0"]
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(x) for x in q]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
# test IK in world coordinates vs base coordinates
rob.BASE=np.zeros(3)
p,R=r.fk(q)
print("world-frame IK:", None if (s:=r.ik(p,R)) is None else (s-q).round(3))
rob.BASE=np.array([-0.66,0,0.912])
print("base-frame IK:", None if (s:=r.ik(p,R)) is None else (s-q).round(3))
EOF
timeout 120 python3 t_ik.py

# openrua op 15
cat > t_ik2.py <<'EOF'
from rob import *
import rob
rob.BASE=np.zeros(3)
r=Rob("t")
q=r.arm_q()
p,R=r.fk(q)
s=r.ik(p,R)
p2,R2=r.fk(s)
print("pos err", (p2-p).round(4)); print("R target\n",R.round(3)); print("R got\n",R2.round(3))
print("sol", s.round(3))
EOF
timeout 120 python3 t_ik2.py

# openrua op 16
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0',
 'BASE = np.zeros(3)  # FK/IK services already answer in WORLD coordinates\n'
 '# IK targets the group tip link panda_link8; panda_hand = link8 * Rz(-45deg)\n'
 'RZ45 = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4), 0],\n'
 '                 [np.sin(np.pi/4), np.cos(np.pi/4), 0], [0, 0, 1]])')
s=s.replace('''        p = p - BASE
        q = R_to_quat(R)''','''        p = p - BASE
        q = R_to_quat(np.asarray(R) @ RZ45)  # hand -> link8 orientation''')
open("rob.py","w").write(s)
EOF
timeout 120 python3 t_ik2.py

# openrua op 17
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_a.png', cv2.resize(im[170:270,150:260],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('frontview.png'); cv2.imwrite('crop_f.png', cv2.resize(im[280:400,380:500],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_s.png', cv2.resize(im[220:380,260:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 18
cat > urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("u"); got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
timeout 30 python3 urdf.py; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -A12 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 19
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(fn):
    data=open(fn,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
    else:
        n=struct.unpack('<I',data[80:84])[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ["hand","finger","link7"]:
    print(f, stl_bbox(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl"))
EOF

# openrua op 20
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -path /proc -prune -o -iname "*moka*" -print 2>/dev/null | head

# openrua op 21
cat > plan.py <<'EOF'
import numpy as np
from rob import *

def hand_R(az_deg, hx_up=False):
    th=np.radians(az_deg)
    hz=np.array([np.cos(th),np.sin(th),0.0])
    hy=np.array([-np.sin(th),np.cos(th),0.0])
    if hx_up: hy=-hy
    return R_from_axes(hz,hy)

A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
ZG=0.962; ZL=1.10; ZP=1.006
A_PLACE=np.array([0.135,-0.015]); B_PLACE=np.array([0.14,0.085])
def u(az): th=np.radians(az); return np.array([np.cos(th),np.sin(th),0.0])
P3=lambda xy,z: np.array([xy[0],xy[1],z])
poses={
 "A_pre":   (P3(A,ZG)-0.12*u(-45), -45),
 "A_grasp": (P3(A,ZG), -45),
 "A_lift":  (P3(A,ZL), -45),
 "A_over":  (P3(A_PLACE,ZL), -75),
 "A_place": (P3(A_PLACE,ZP), -75),
 "B_pre":   (P3(B,ZG)-0.12*u(-135), -135),
 "B_grasp": (P3(B,ZG), -135),
 "B_lift":  (P3(B,ZL), -135),
 "B_over":  (P3(B_PLACE,ZL), -60),
 "B_place": (P3(B_PLACE,ZP), -60),
}
if __name__=="__main__":
    r=Rob("plan")
    q0=r.arm_q()
    for name,(p,az) in poses.items():
        for hx_up in (False,True):
            q=r.ik(p,hand_R(az,hx_up),seed=q0,at_tcp=True)
            if q is None: print(f"{name} hx_up={hx_up}: IK FAIL"); continue
            # check link heights
            zs={l: r.fk(q,l)[0][2] for l in ["panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]}
            pt,_=r.tcp(q)
            print(f"{name} hx_up={hx_up}: q={q.round(2)} tcp={pt.round(3)} minlinkz={min(zs.values()):.3f} ({min(zs,key=zs.get)})")
EOF
timeout 600 python3 plan.py

# openrua op 22
cat > plan.py <<'EOF'
import numpy as np
from rob import *

def u(az):
    th=np.radians(az); return np.array([np.cos(th),np.sin(th),0.0])
def hand_R(az_deg, hx_up=False):
    hz=u(az_deg); hy=np.array([-hz[1],hz[0],0.0])
    if hx_up: hy=-hy
    return R_from_axes(hz,hy)
P3=lambda xy,z: np.array([xy[0],xy[1],z])

A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
ZG=0.955; ZL=1.10; ZP=1.02
A_PLACE=np.array([0.13,-0.018]); B_PLACE=np.array([0.14,0.085])
poses={
 "A_pre":   (P3(A,ZG)-0.12*u(-45), -45),
 "A_grasp": (P3(A,ZG), -45),
 "A_lift":  (P3(A,ZL), -45),
 "A_liftrot":(P3(A,ZL), 45),
 "A_downrot":(P3(A,ZG), 45),
 "A_over":  (P3(A_PLACE,ZL), -75),
 "A_place": (P3(A_PLACE,ZP), -75),
 "B_pre":   (P3(B,ZG)-0.12*u(-45), -45),
 "B_grasp": (P3(B,ZG), -45),
 "B_lift":  (P3(B,ZL), -45),
 "B_liftrot":(P3(B,ZL), 45),
 "B_downrot":(P3(B,ZG), 45),
 "B_over":  (P3(B_PLACE,ZL), -60),
 "B_place": (P3(B_PLACE,ZP), -60),
}
if __name__=="__main__":
    r=Rob("plan")
    q0=r.arm_q()
    for name,(p,az) in poses.items():
        for hx_up in (False,True):
            q=r.ik(p,hand_R(az,hx_up),seed=q0,at_tcp=True)
            if q is None: print(f"{name} hx_up={hx_up}: IK FAIL"); continue
            zs={l: r.fk(q,l)[0][2] for l in ["panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]}
            pt,_=r.tcp(q)
            print(f"{name} hx_up={hx_up}: q={q.round(2)} tcp={pt.round(3)} minlinkz={min(zs.values()):.3f} ({min(zs,key=zs.get)})")
EOF
timeout 900 python3 plan.py

# openrua op 23
cat > yawtest.py <<'EOF'
import numpy as np
from rob import *
from plan import *
r=Rob("yaw")
for name,xy in (("A",A),("B",B)):
    q=r.ik(P3(xy,ZL),hand_R(-45),seed=r.arm_q(),at_tcp=True)
    print(name,"start",q.round(2))
    prev=q
    for az in range(-35,50,10):
        q=r.ik(P3(xy,ZL),hand_R(az),seed=prev,at_tcp=True)
        if q is None: print(f"  az {az}: FAIL"); break
        print(f"  az {az}: q={q.round(2)} dq_max={np.abs(q-prev).max():.2f}")
        prev=q
EOF
timeout 900 python3 yawtest.py

# openrua op 24
ls *.npy && cat > handle.py <<'EOF'
import numpy as np
A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
for f in ("bird_world.npy","side_world.npy"):
    P=np.load(f).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    for name,c in (("A",A),("B",B)):
        d=P[:,:2]-c; r=np.linalg.norm(d,axis=1); az=np.degrees(np.arctan2(d[:,1],d[:,0]))
        m=(r<0.12)&(P[:,2]>0.90)
        print(f,name,m.sum())
        # for az bins of 15 deg and z bins of 1cm print max r
        zs=np.arange(0.90,1.07,0.01)
        print("      "+" ".join(f"{a:4d}" for a in range(-180,180,15)))
        for z0 in zs:
            row=[]
            for a0 in range(-180,180,15):
                mm=m&(P[:,2]>=z0)&(P[:,2]<z0+0.01)&(az>=a0)&(az<a0+15)
                row.append(f"{(r[mm].max()*100 if mm.any() else 0):4.1f}")
            print(f"{z0:.2f} "+" ".join(row))
EOF
python3 handle.py

# openrua op 25
cat > stovemap.py <<'EOF'
import numpy as np
P=np.load("bird_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
xs=np.arange(0.07,0.29,0.01); ys=np.arange(-0.08,0.14,0.01)
print("z(mm above 0.9) rows=y (top=+y), cols=x")
print("      "+" ".join(f"{x*100:3.0f}" for x in xs))
for y0 in ys[::-1]:
    row=[]
    for x0 in xs:
        m=(P[:,0]>=x0)&(P[:,0]<x0+0.01)&(P[:,1]>=y0)&(P[:,1]<y0+0.01)
        row.append(f"{(np.median(P[m,2])-0.9)*1000:3.0f}" if m.sum()>2 else "  .")
    print(f"{y0*100:4.0f}  "+" ".join(row))
EOF
python3 stovemap.py

# openrua op 26
grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'collision' robot.urdf | grep -A10 'panda_link7\|panda_hand\|finger' | grep -E 'link name|geometry|box|cylinder|sphere|mesh|origin' | head -40

# openrua op 27
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>&1 | head; python3 - <<'EOF'
import numpy as np, struct, os
for n in ("hand","link7","finger"):
    f=f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{n}.stl"
    if not os.path.exists(f): print("missing",f); continue
    b=open(f,"rb").read()
    if b[:5]==b"solid":
        V=np.array([[float(x) for x in l.split()[1:]] for l in b.decode().splitlines() if l.strip().startswith("vertex")])
    else:
        n_=struct.unpack("<I",b[80:84])[0]
        arr=np.frombuffer(b[84:84+n_*50],dtype=np.dtype([("n","<3f"),("v","<9f"),("a","<H")]))
        V=arr["v"].reshape(-1,3)
    print(n, "min",V.min(0).round(4),"max",V.max(0).round(4))
EOF

# openrua op 28
cat > reachtest.py <<'EOF'
import numpy as np
from rob import *
from plan import hand_R, P3, u
r=Rob("reach")
seed0=np.array([-0.24,0.71,0.07,-2.51,-2.54,1.48,0.79])  # A_grasp-like (q5<0 branch)
for xy in ((0.225,0.05),(0.23,0.05),(0.235,0.05),(0.225,0.07)):
    for az in (-45,-35,-25,-15):
        for z in (1.014,1.10):
            q=r.ik(P3(xy,z),hand_R(az),seed=seed0,at_tcp=True)
            print(xy,az,z, None if q is None else q.round(2))
EOF
grep -n "def P3\|^def " plan.py | head; timeout 600 python3 reachtest.py

# openrua op 29
sed -n 1,20p plan.py

# openrua op 30
cat > plan2.py <<'EOF'
import numpy as np
from rob import *

def u(az):
    th=np.radians(az); return np.array([np.cos(th),np.sin(th),0.0])
def hand_R(az_deg):
    hz=u(az_deg); hy=np.array([-hz[1],hz[0],0.0])
    return R_from_axes(hz,hy)
P3=lambda xy,z: np.array([xy[0],xy[1],z])

A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
ZG=0.965          # TCP height at grasp (waist centre); pot bottom is 0.071 below
ZL=1.15           # transport height (pot bottom 1.08 > other pot top 1.057)
ZP=1.014          # release height (pot bottom ~1.0 cm above burner grate 0.933)
FAR=np.array([0.23,0.05]);  TH_FAR=-30
NEAR=np.array([0.125,0.05]); TH_NEAR=-45
PRE=0.12; RET=0.10

def seq_pick(c, th):
    return [("pre_high", P3(c,ZL)-PRE*u(th), th),
            ("pre",      P3(c,ZG)-PRE*u(th), th),
            ("grasp",    P3(c,ZG), th),
            ("lift",     P3(c,ZL), th)]
def seq_place(c, th):
    return [("over",   P3(c,ZL), th),
            ("place",  P3(c,ZP), th),
            ("retreat",P3(c,ZP)-RET*u(th), th),
            ("up",     P3(c,ZL)-RET*u(th), th)]
SEQ = ([("A_"+n,p,t) for n,p,t in seq_pick(A,-45)] +
       [("A_"+n,p,t) for n,p,t in seq_place(FAR,TH_FAR)] +
       [("B_"+n,p,t) for n,p,t in seq_pick(B,-45)] +
       [("B_"+n,p,t) for n,p,t in seq_place(NEAR,TH_NEAR)])

if __name__=="__main__":
    r=Rob("plan2")
    prev=r.arm_q(); print("current", prev.round(2))
    lim=np.array(M["actuators"][0]["limits_rad"])
    sols={}
    for name,p,th in SEQ:
        q=r.ik(p,hand_R(th),seed=prev,at_tcp=True)
        if q is None:
            print(f"{name:10s} FAIL"); continue
        dq=np.abs(q-prev).max(); marg=np.minimum(q-lim[:,0],lim[:,1]-q).min()
        print(f"{name:10s} q={q.round(2)} dq_max={dq:.2f} lim_margin={marg:.2f}")
        prev=q; sols[name]=q
    np.save("sols.npy", sols, allow_pickle=True)
EOF
timeout 900 python3 plan2.py

# openrua op 31
cat > pathcheck.py <<'EOF'
import numpy as np
from rob import *
from plan2 import *
r=Rob("pathcheck")
sols=np.load("sols.npy",allow_pickle=True).item()
names=[n for n,_,_ in SEQ]
prev_n="ready"; prev_q=r.arm_q()
for n in names:
    q=sols[n]; worst=(0,0,9); 
    for s in np.linspace(0,1,9)[1:-1]:
        qi=prev_q+(q-prev_q)*s
        p,R=r.tcp(qi)
        tilt=np.degrees(np.arcsin(abs(R[2,2])))        # approach axis off horizontal
        roll=np.degrees(np.arccos(np.clip(-R[2,0],-1,1)))  # hand x vs straight down
        worst=(max(worst[0],tilt),max(worst[1],roll),min(worst[2],p[2]))
    print(f"{prev_n:10s}->{n:10s} max_tilt={worst[0]:5.1f} max_roll={worst[1]:5.1f} min_z={worst[2]:.3f}")
    prev_n,prev_q=n,q
EOF
timeout 900 python3 pathcheck.py

# openrua op 32
cat > exec.py <<'EOF'
import sys, numpy as np
from rob import *
from plan2 import *
pot=sys.argv[1]           # "A" or "B"
r=Rob("exec_"+pot)
sols=np.load("sols.npy",allow_pickle=True).item()
DUR={"pre_high":4.0,"pre":3.0,"grasp":2.5,"lift":3.0,"over":5.0,"place":3.0,"retreat":2.5,"up":2.5}
if pot=="A": DUR["pre_high"]=6.0   # big unloaded reconfiguration from ready pose

def pot_blob(c):
    P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=np.linalg.norm(P[:,:2]-c,axis=1); m=(d<0.06)&(P[:,2]>0.95)&(P[:,2]<1.08)
    if m.sum()<5: return None
    return P[m,:2].mean(0).round(3), P[m,2].max().round(3), int(m.sum())

def go(step, seed_name):
    q=sols[f"{pot}_{step}"]
    code,err=r.move_q(q, DUR[step])
    p,R=r.tcp(); tgt=dict((n,(pp,t)) for n,pp,t in SEQ)[f"{pot}_{step}"][0]
    print(f"== {pot}_{step}: code={code} jerr={err:.4f} tcp={p.round(3)} tgt={tgt.round(3)} "
          f"poserr={np.linalg.norm(p-tgt)*1000:.1f}mm fingers={np.round(r.fingers(),4)} F={r.wrench()[:3].round(1)}", flush=True)
    if code!=0 or err>0.02: raise SystemExit("MOVE FAILED")

src = A if pot=="A" else B
dst = FAR if pot=="A" else NEAR
print("pot at start:", pot_blob(src), " dst:", pot_blob(dst), flush=True)
f=r.gripper(0.04)
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre_high", None); go("pre", None); go("grasp", None)
print("pot before close:", pot_blob(src), flush=True)
f=r.gripper(0.0)
print("fingers after close:", f, flush=True)
if not (0.022<min(f)<0.038): raise SystemExit("GRASP MISSED (fingers %s)"%(f,))
go("lift", None)
f=r.fingers(); print("fingers after lift:", f, " src blob now:", pot_blob(src), flush=True)
if min(f)<0.022: raise SystemExit("POT DROPPED")
go("over", None); go("place", None)
f=r.gripper(0.04)
go("retreat", None); go("up", None)
print("RESULT dst blob:", pot_blob(dst), " src blob:", pot_blob(src), flush=True)
for cam in ("birdview","agentview","frontview"):
    r.snap(cam, f"/workspace/after_{pot}_{cam}.png")
print("DONE", flush=True)
EOF
python3 -u exec.py A > exec_A.log 2>&1 &
sleep 60; cat exec_A.log

# openrua op 33
python3 - <<'EOF'
s=open("exec.py").read()
s=s.replace('f=r.gripper(0.04)\nif min(f)<0.035', 'f=np.abs(r.gripper(0.04))\nif min(f)<0.035')
s=s.replace('f=r.gripper(0.0)\n', 'f=np.abs(r.gripper(0.0))\n')
s=s.replace('f=r.fingers(); print("fingers after lift:"', 'f=np.abs(r.fingers()); print("fingers after lift:"')
s=s.replace('fingers={np.round(r.fingers(),4)}', 'fingers={np.round(np.abs(r.fingers()),4)}')
open("exec.py","w").write(s)
EOF
grep -n "abs" exec.py; python3 -u exec.py A > exec_A.log 2>&1 &
sleep 90; cat /workspace/exec_A.log

# openrua op 34
grep -n -i "tolerance\|-5\|abort\|collision" docs/*.md | head -20; python3 - <<'EOF'
import numpy as np
from rob import *
r=Rob("chk"); q=r.arm_q(); print("q",q.round(3)); p,R=r.tcp(); print("tcp",p.round(3)); print("F",r.wrench().round(2)); print("fingers",r.fingers())
EOF

# openrua op 35
cat > retry.py <<'EOF'
import numpy as np, time
from rob import *
r=Rob("retry"); sols=np.load("sols.npy",allow_pickle=True).item()
q=sols["A_pre_high"]
for i in range(3):
    t0=time.time(); code,err=r.move_q(q,4.0); print("wall",round(time.time()-t0,1),"q",r.arm_q().round(3))
    if code==0: break
EOF
timeout 600 python3 -u retry.py

# openrua op 36
python3 - <<'EOF'
s=open("rob.py").read()
old='''        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"[move_q] error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err'''
new='''        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"[move_q] error_code={code} max_joint_err={err:.4f}", flush=True)
        # -5 = goal tolerance violated: this controller caps joint speed
        # (~0.19 rad/s); resending the same goal converges (docs/30-action)
        if code == -5 and retries > 0 and err > 0.01:
            return self.move_q(q, max(2.0, err / 0.15), retries=retries - 1)
        return code, err'''
assert old in s
s=s.replace(old,new).replace("def move_q(self, q, seconds=3.0, waypoints=None):","def move_q(self, q, seconds=3.0, waypoints=None, retries=3):")
open("rob.py","w").write(s)
e=open("exec.py").read()
old2='''    q=sols[f"{pot}_{step}"]
    code,err=r.move_q(q, DUR[step])'''
new2='''    q=sols[f"{pot}_{step}"]
    dur=max(2.0, np.abs(q-r.arm_q()).max()/0.15)   # joint speed cap ~0.19 rad/s
    code,err=r.move_q(q, dur)'''
assert old2 in e
open("exec.py","w").write(e.replace(old2,new2))
EOF
python3 -u exec.py A > exec_A.log 2>&1 &
sleep 240; cat /workspace/exec_A.log

# openrua op 37
python3 - <<'EOF'
import numpy as np
from rob import *
r=Rob("chk"); print("fingers",r.fingers()); print("F",r.wrench().round(2)); print("tcp",r.tcp()[0].round(3))
EOF
grep -n -i -B3 -A12 "settle\|gripper" docs/40-patterns.md | head -80

# openrua op 38
python3 - <<'EOF'
from rob import *
r=Rob("snap")
for c in ("robot0_eye_in_hand","agentview","sideview","frontview"): r.snap(c, f"/workspace/g_{c}.png")
EOF
echo ok

# openrua op 39
cat > prof2.py <<'EOF'
import numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("prof2"); buf=Buffer(); TransformListener(buf,r.node)
B=np.array([-0.064,0.234])
for cam in ("sideview","frontview","agentview"):
    fr=f"{cam}_optical_frame"
    for _ in range(50):
        r.spin(0.1)
        if buf.can_transform("world",fr,rclpy.time.Time()): break
    t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
    tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
    print(cam,"cam pos",np.round(tt,3))
    P=r.depth_world(cam,tt,q).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=P[:,:2]-B; m=(np.linalg.norm(d,axis=1)<0.08)&(P[:,2]>0.895)&(P[:,2]<1.07)
    Q=P[m]; dd=d[m]
    # view direction in xy from camera to pot; lateral axis perpendicular
    v=B-np.array(tt[:2]); v/=np.linalg.norm(v); lat=np.array([-v[1],v[0]])
    l=dd@lat; a=dd@v
    for z0 in np.arange(0.895,1.06,0.005):
        mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)
        if mm.sum()>3:
            print(f"  z={z0:.3f} width={100*(l[mm].max()-l[mm].min()):4.1f}cm  near_face_dist={100*a[mm].min():5.1f}  n={mm.sum()}")
EOF
timeout 300 python3 prof2.py 2>&1 | head -120

# openrua op 40
cat > eyeprof.py <<'EOF'
import numpy as np, rclpy, sys
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("eyeprof"); buf=Buffer(); TransformListener(buf,r.node)
sols=np.load("sols.npy",allow_pickle=True).item()
if "back" in sys.argv:
    r.gripper(0.04); r.move_q(sols["A_pre"],3.0)
cam="robot0_eye_in_hand"; fr=f"{cam}_optical_frame"
for _ in range(50):
    r.spin(0.1)
    if buf.can_transform("world",fr,rclpy.time.Time()): break
t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
print("cam pos",np.round(tt,3),"tcp",r.tcp()[0].round(3))
P=r.depth_world(cam,tt,q); np.save("eye_world.npy",P); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=np.array([-0.195,-0.200])
d=P[:,:2]-A; rr=np.linalg.norm(d,axis=1); m=(rr<0.09)&(P[:,2]>0.895)&(P[:,2]<1.07)
Q=P[m]; dd=d[m]; az=np.degrees(np.arctan2(dd[:,1],dd[:,0]))
print("n",m.sum())
for z0 in np.arange(0.895,1.06,0.005):
    mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)
    if mm.sum()>3:
        # radius in the visible az range, and lateral width along 45deg axis (finger axis)
        lat=dd[mm]@np.array([0.7071,0.7071]); ap=dd[mm]@np.array([0.7071,-0.7071])
        print(f"  z={z0:.3f} n={mm.sum():4d} r_max={100*rr[m][mm].max():4.1f} lat=[{100*lat.min():5.1f},{100*lat.max():5.1f}] near_face={100*ap.min():5.1f} az=[{az[mm].min():4.0f},{az[mm].max():4.0f}]")
EOF
timeout 300 python3 eyeprof.py back 2>&1 | tail -45

# openrua op 41
cat > rgbprof.py <<'EOF'
import numpy as np, rclpy, cv2
from rob import *
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
r=Rob("rgbprof"); buf=Buffer(); TransformListener(buf,r.node)
cam="sideview"; fr=f"{cam}_optical_frame"
for _ in range(50):
    r.spin(0.1)
    if buf.can_transform("world",fr,rclpy.time.Time()): break
t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
tt=np.array([t.translation.x,t.translation.y,t.translation.z]); q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
img=cv2.imread(r.snap(cam,"/workspace/sv.png"))
P=r.depth_world(cam,tt,q)
B=np.array([-0.064,0.234])
# pot pixels: world points within 6cm of B in xy and above table
d=np.linalg.norm(P[:,:,:2]-B,axis=2); mask=(d<0.06)&(P[:,:,2]>0.9)&(P[:,:,2]<1.06)
vs,us=np.where(mask); print("depth-mask bbox u",us.min(),us.max(),"v",vs.min(),vs.max())
# RGB segmentation inside a padded bbox: not wood (wood is warm: R>G>B strongly)
u0,u1,v0,v1=us.min()-15,us.max()+15,vs.min()-10,vs.max()+10
sub=img[v0:v1,u0:u1].astype(int); b,g,rr=sub[...,0],sub[...,1],sub[...,2]
wood=(rr-b>25)                       # table is orange-ish
pot=~wood
Rw=quat_to_R(q); fx=None
from sensor_msgs.msg import CameraInfo
got={}
sub_=r.node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("m",m),1)
while "m" not in got: r.spin(0.2)
fx=got["m"].k[0]; fy=got["m"].k[4]; cx=got["m"].k[2]; cy=got["m"].k[5]
Zc=np.linalg.norm(np.array([B[0],B[1],0.98])-tt)   # distance to pot centre
print("fx",fx,"Zc",Zc, "mm/px", 1000*Zc/fx)
cv2.imwrite("/workspace/sv_crop.png", np.where(pot[...,None],sub,sub//3).astype(np.uint8))
for v in range(0,v1-v0):
    row=pot[v]; 
    if row.sum()<3: continue
    uu=np.where(row)[0]; w=(uu.max()-uu.min()+1)*Zc/fx
    zrow=P[v0+v, u0+uu.min():u0+uu.max()+1, 2]; zrow=zrow[np.isfinite(zrow)]
    zz=np.median(zrow) if zrow.size else float('nan')
    print(f"row {v0+v}: width={100*w:4.1f}cm  z~{zz:.3f}  u=[{u0+uu.min()},{u0+uu.max()}]")
EOF
timeout 300 python3 rgbprof.py 2>&1 | tail -70

# openrua op 42
timeout 300 python3 - <<'EOF'
import numpy as np
from rob import *
r=Rob("gt"); print("tcp",r.tcp()[0].round(3),"fingers",r.fingers())
r.gripper(0.0); print("after close:",r.fingers())
r.gripper(0.04); print("after open:",r.fingers())
EOF

# openrua op 43
timeout 120 python3 - <<'EOF'
import numpy as np
P=np.load("eye_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=np.array([-0.195,-0.200])
for z0,z1 in ((0.905,0.945),(0.958,0.972),(0.98,1.0)):
    m=(P[:,2]>=z0)&(P[:,2]<z1)&(np.abs(P[:,0]-A[0])<0.1)&(np.abs(P[:,1]-A[1])<0.1)
    Q=P[m]; print(f"z in [{z0},{z1}) n={m.sum()}  (grid 5mm, x right, y up)")
    g=np.zeros((40,40),int)
    ix=((Q[:,0]-A[0]+0.1)/0.005).astype(int).clip(0,39); iy=((Q[:,1]-A[1]+0.1)/0.005).astype(int).clip(0,39)
    np.add.at(g,(iy,ix),1)
    for row in g[::-1]:
        print("".join("#" if c>3 else ("." if c>0 else " ") for c in row))
EOF

# openrua op 44
timeout 120 python3 - <<'EOF'
import numpy as np
P=np.load("eye_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
A=np.array([-0.195,-0.200])
def fit(Q):
    # algebraic circle fit
    x,y=Q[:,0],Q[:,1]; M=np.c_[2*x,2*y,np.ones(len(x))]; b=x*x+y*y
    c=np.linalg.lstsq(M,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx*cx+cy*cy); return cx,cy,r
for z0 in np.arange(0.90,1.03,0.005):
    m=(P[:,2]>=z0)&(P[:,2]<z0+0.005)&(np.abs(P[:,0]-A[0])<0.06)&(np.abs(P[:,1]-A[1])<0.06)
    Q=P[m]
    if len(Q)<20: continue
    d=Q[:,:2]-A; az=np.degrees(np.arctan2(d[:,1],d[:,0]))
    body=Q[(az>60)&(az<200)|(az<-160)]   # visible body face, exclude handle(-90) and spout(+90)... keep 60..200
    body=Q[((az>100)&(az<=180))|((az>=-180)&(az<-150))]
    if len(body)<15: continue
    cx,cy,r=fit(body[:,:2]); res=np.abs(np.linalg.norm(body[:,:2]-[cx,cy],axis=1)-r).mean()
    print(f"z={z0:.3f} n={len(body):4d} centre=({cx:.4f},{cy:.4f}) r={100*r:4.2f}cm  resid={1000*res:.1f}mm")
EOF

# openrua op 45
cat > probe.py <<'EOF'
import numpy as np, rclpy, sys
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("probe"); buf=Buffer(); TransformListener(buf,r.node)
sols=np.load("sols.npy",allow_pickle=True).item()
cam="robot0_eye_in_hand"; fr=f"{cam}_optical_frame"
def cloud(tag):
    for _ in range(50):
        r.spin(0.1)
        if buf.can_transform("world",fr,rclpy.time.Time()): break
    t=buf.lookup_transform("world",fr,rclpy.time.Time()).transform
    tt=[t.translation.x,t.translation.y,t.translation.z]; q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
    P=r.depth_world(cam,tt,q); np.save(f"eye_{tag}.npy",P); return P
r.move_q(sols["A_grasp"],3.0); print("tcp",r.tcp()[0].round(4),"fingers",r.fingers(),flush=True)
cloud("open")
r.gripper(0.0); print("closed fingers",r.fingers(),flush=True)
cloud("closed")
r.snap(cam,"/workspace/eye_closed.png")
r.gripper(0.04); r.move_q(sols["A_pre"],3.0); print("back at pre",r.fingers(),flush=True)
EOF
timeout 600 python3 -u probe.py

# openrua op 46
timeout 120 python3 - <<'EOF'
import numpy as np
T=np.array([-0.1952,-0.1988,0.9653])
for tag in ("open","closed"):
    P=np.load(f"eye_{tag}.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    print("=====",tag)
    for z0,z1 in ((0.94,0.955),(0.955,0.975),(0.975,0.99)):
        m=(P[:,2]>=z0)&(P[:,2]<z1)&(np.abs(P[:,0]-T[0])<0.1)&(np.abs(P[:,1]-T[1])<0.1)
        Q=P[m]; print(f"z in [{z0},{z1}) n={m.sum()}  (grid 5mm, x right, y up, centre=TCP)")
        g=np.zeros((40,40),int)
        ix=((Q[:,0]-T[0]+0.1)/0.005).astype(int).clip(0,39); iy=((Q[:,1]-T[1]+0.1)/0.005).astype(int).clip(0,39)
        np.add.at(g,(iy,ix),1)
        for i,row in enumerate(g[::-1]):
            print("".join("#" if c>3 else ("." if c>0 else " ") for c in row), "<" if i==20 else "")
EOF

# openrua op 47
timeout 120 python3 - <<'EOF'
import numpy as np
T=np.array([-0.1952,-0.1988,0.9653])
u=lambda d: np.array([np.cos(np.radians(d)),np.sin(np.radians(d))])
P=np.load("eye_open.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
d=P[:,:2]-T[:2]; a=d@u(-45); l=d@u(45)
for side,name in ((1,"finger +l (az45 side)"),(-1,"finger -l (az-135 side)")):
    m=(side*l>0.041)&(side*l<0.06)&(a>-0.06)&(a<0.012)&(P[:,2]>0.9)&(P[:,2]<1.05)
    print(name,"n",m.sum())
    h,e=np.histogram(P[m,2],bins=np.arange(0.90,1.05,0.005))
    for c,z0 in zip(h,e[:-1]):
        if c: print(f"   z={z0:.3f} n={c:5d}  l_range=[{100*(side*l[m][(P[m,2]>=z0)&(P[m,2]<z0+0.005)]).min():.1f},{100*(side*l[m][(P[m,2]>=z0)&(P[m,2]<z0+0.005)]).max():.1f}] a_range=[{100*a[m][(P[m,2]>=z0)&(P[m,2]<z0+0.005)].min():.1f},{100*a[m][(P[m,2]>=z0)&(P[m,2]<z0+0.005)].max():.1f}]")
# pot points near the finger axis: |l| in [0.028,0.041], z bands
m=(np.abs(l)>0.028)&(np.abs(l)<0.041)&(np.abs(a)<0.02)
print("pot near finger axis"); h,e=np.histogram(P[m,2],bins=np.arange(0.90,1.05,0.005))
for c,z0 in zip(h,e[:-1]):
    if c: print(f"   z={z0:.3f} n={c:5d} |l|max={100*np.abs(l[m][(P[m,2]>=z0)&(P[m,2]<z0+0.005)]).max():.2f}")
EOF

# openrua op 48
sed -i 's/^ZG=0.965.*/ZG=0.975          # TCP height at grasp: finger band [TCP-0.025,TCP-0.004] sits in waist 0.950-0.971; pot bottom 0.081 below TCP/; s/^ZL=1.15.*/ZL=1.17           # transport height (pot bottom 1.089 > other pot top 1.057)/; s/^ZP=1.014.*/ZP=1.026          # release height (pot bottom ~1.2 cm above burner grate 0.933)/' plan2.py
grep -n "^Z" plan2.py
sed -i 's/if not (0.022<min(f)<0.038)/if not (0.022<min(f)<0.036)/' exec.py
timeout 900 python3 plan2.py && timeout 900 python3 pathcheck.py

# openrua op 49
python3 -u exec.py A > exec_A.log 2>&1 &
sleep 200; cat /workspace/exec_A.log

# openrua op 50
cat > lifttest.py <<'EOF'
import numpy as np
from rob import *
from plan2 import *
r=Rob("lifttest"); sols=np.load("sols.npy",allow_pickle=True).item()
def blob(c):
    P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=np.linalg.norm(P[:,:2]-c,axis=1); m=(d<0.06)&(P[:,2]>0.95)&(P[:,2]<1.2)
    return (P[m,:2].mean(0).round(3), P[m,2].min().round(3), P[m,2].max().round(3), int(m.sum())) if m.sum()>5 else None
print("fingers",r.fingers(),"tcp",r.tcp()[0].round(3))
# lift just 3 cm first
q=r.ik(P3(A,ZG+0.03),hand_R(-45),seed=r.arm_q(),at_tcp=True)
r.move_q(q,2.5); print("after +3cm: fingers",np.abs(r.fingers()).round(4),"blob",blob(A),"F",r.wrench()[:3].round(2))
r.snap("agentview","/workspace/lift3.png")
EOF
timeout 600 python3 -u lifttest.py

# openrua op 51
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/exec.py", "old_string": "src = A if pot==\\"A\\" else B\\ndst = FAR if pot==\\"A\\" else NEAR\\nprint(\\"pot at start:\\", pot_blob(src), \\" dst:\\", pot_blob(dst), flush=True)\\nf=np.abs(r.gripper(0.04))\\nif min(f)<0.035: raise SystemExit(\\"gripper did not open\\")\\ngo(\\"pre_high\\", None); go(\\"pre\\", None); go(\\"grasp\\", None)\\nprint(\\"pot before close:\\", pot_blob(src), flush=True)\\nf=np.abs(r.gripper(0.0))\\nprint(\\"fingers after close:\\", f, flush=True)\\nif not (0.022<min(f)<0.036): raise SystemExit(\\"GRASP MISSED (fingers %s)\\"%(f,))\\ngo(\\"lift\\", None)", "new_string": "src = A if pot==\\"A\\" else B\\ndst = FAR if pot==\\"A\\" else NEAR\\nresume = len(sys.argv)>2 and sys.argv[2]==\\"held\\"   # pot already in gripper: skip the pick\\nprint(\\"pot at start:\\", pot_blob(src), \\" dst:\\", pot_blob(dst), flush=True)\\nif not resume:\\n    f=np.abs(r.gripper(0.04))\\n    if min(f)<0.035: raise SystemExit(\\"gripper did not open\\")\\n    go(\\"pre_high\\", None); go(\\"pre\\", None); go(\\"grasp\\", None)\\n    print(\\"pot before close:\\", pot_blob(src), flush=True)\\n    f=np.abs(r.gripper(0.0))\\n    print(\\"fingers after close:\\", f, flush=True)\\n    # holding the pot\'s waist ring reads ~0.037 per finger; a miss closes to ~0.004\\n    if not (0.022<min(f)<0.0395): raise SystemExit(\\"GRASP MISSED (fingers %s)\\"%(f,))\\ngo(\\"lift\\", None)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 52
python3 -u exec.py A held > exec_A2.log 2>&1 &
sleep 60; cat /workspace/exec_A2.log

# openrua op 53
sleep 75; cat /workspace/exec_A2.log

# openrua op 54
cat > /workspace/state.py <<'EOF'
import numpy as np
from rob import *
r=Rob("state")
P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
np.save("bird_now.npy",P)
def region(name,xr,yr,zmin=0.94):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)
    if m.sum()==0: print(name,"empty"); return
    Q=P[m]; print(f"{name}: n={m.sum()} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] zmax={Q[:,2].max():.3f} top-pt={Q[Q[:,2].argmax()].round(3)}")
    for z in np.arange(zmin,Q[:,2].max(),0.02):
        mm=(Q[:,2]>z)&(Q[:,2]<z+0.02)
        if mm.sum(): print(f"   z{z:.2f}: n={mm.sum()} x[{Q[mm,0].min():.3f},{Q[mm,0].max():.3f}] y[{Q[mm,1].min():.3f},{Q[mm,1].max():.3f}]")
region("B", (-0.2,0.05),(0.12,0.35))
region("stove",(0.06,0.30),(-0.08,0.15),0.94)
region("A_orig",(-0.3,-0.1),(-0.3,-0.1))
print("tcp",r.tcp()[0].round(3),"fingers",r.fingers())
EOF
python3 state.py 2>&1 | tail -40

# openrua op 55
grep -n "ready\|0.785\|-2.356\|1.571" exec_A.log docs/*.md rob.py plan2.py 2>/dev/null | head; python3 - <<'EOF'
import json,sys
# find the first joint state recorded in earlier logs
import glob
for f in glob.glob("/workspace/*.log"):
    print(f); print(open(f).read()[:600]); print("----")
EOF

# openrua op 56
cat > /workspace/park.py <<'EOF'
import numpy as np, sys
from rob import *
r=Rob("park")
cands=[[0.31,-0.3,-0.06,-2.4,-2.13,2.16,0.12],[0.31,-0.6,-0.06,-2.6,-2.13,2.0,0.12],[0.0,-0.8,0.0,-2.5,0.0,1.7,0.785],[0.31,-0.6,-0.06,-2.6,-2.13,2.5,0.12]]
for q in cands:
    p,R=r.tcp(np.array(q)); zs={l:r.fk(np.array(q),l)[0][2] for l in ["panda_link4","panda_link6","panda_hand"]}
    print(q,"tcp",p.round(3),"hz",R[:,2].round(2),{k:round(v,3) for k,v in zs.items()})
if len(sys.argv)>1:
    q=np.array(cands[int(sys.argv[1])]); dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15)
    print(r.move_q(q,dur)); print("tcp",r.tcp()[0].round(3))
EOF
python3 park.py 2>&1 | tail -5

# openrua op 57
python3 park.py 2 > park.log 2>&1; tail -3 park.log; python3 state.py 2>&1 | tail -30

# openrua op 58
cat > /workspace/state2.py <<'EOF'
import numpy as np
from rob import *
r=Rob("state2")
P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
np.save("bird_now.npy",P)
def blob(name,xr,yr,zmin):
    m=(P[:,0]>xr[0])&(P[:,0]<xr[1])&(P[:,1]>yr[0])&(P[:,1]<yr[1])&(P[:,2]>zmin)
    Q=P[m]; c=Q[:,:2].mean(0); U,S,Vt=np.linalg.svd(Q[:,:2]-c,full_matrices=False)
    ax=Vt[0]; proj=(Q[:,:2]-c)@ax; perp=(Q[:,:2]-c)@Vt[1]
    print(f"{name}: n={len(Q)} center={c.round(3)} axis={ax.round(3)} (az {np.degrees(np.arctan2(ax[1],ax[0])):.0f}) len={proj.max()-proj.min():.3f} width={perp.max()-perp.min():.3f} zmax={Q[:,2].max():.3f}")
    for s in np.arange(proj.min(),proj.max(),0.015):
        mm=(proj>=s)&(proj<s+0.015)
        if mm.sum(): print(f"   along {s:+.3f}: n={mm.sum():3d} zmax={Q[mm,2].max():.3f} width={perp[mm].max()-perp[mm].min():.3f} perp_c={perp[mm].mean():+.3f}")
blob("B",(-0.35,0.05),(0.1,0.45),0.91)
blob("A",(0.05,0.35),(-0.1,0.2),0.94)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/now_{cam}.png")
EOF
python3 state2.py 2>&1 | tail -40

# openrua op 59
cat > /workspace/hover.py <<'EOF'
import sys, numpy as np, cv2
from rob import *
r=Rob("hover")
x,y,z=map(float,sys.argv[1:4]); hyaz=float(sys.argv[4]) if len(sys.argv)>4 else 90.0
tag=sys.argv[5] if len(sys.argv)>5 else "hov"
hz=np.array([0,0,-1.0]); hy=np.array([np.cos(np.radians(hyaz)),np.sin(np.radians(hyaz)),0])
R=R_from_axes(hz,hy)
q=r.ik([x,y,z],R,at_tcp=True)
if q is None: raise SystemExit("IK FAIL")
print("q",q.round(3))
dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15)
print(r.move_q(q,dur)); p,Rn=r.tcp(); print("tcp",p.round(3),"hz",Rn[:,2].round(2),"hy",Rn[:,1].round(2))
r.snap("robot0_eye_in_hand",f"/workspace/{tag}_eye.png")
# depth -> world via TF
from tf2_ros import Buffer, TransformListener
import rclpy
buf=Buffer(); TransformListener(buf,r.node)
fr="robot0_eye_in_hand_optical_frame"
while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
P=r.depth_world("robot0_eye_in_hand",[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w])
np.save(f"/workspace/{tag}_eye_world.npy",P)
print("cam at",np.round([tr.x,tr.y,tr.z],3))
EOF
python3 hover.py -0.167 0.277 1.20 90 hovB 2>&1 | tail -4

# openrua op 60
cat > /workspace/lying.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([float(sys.argv[2]),float(sys.argv[3])]); zmin=float(sys.argv[4]) if len(sys.argv)>4 else 0.90
m=(np.linalg.norm(P[:,:2]-c,axis=1)<0.16)&(P[:,2]>zmin)&(P[:,2]<1.15)
Q=P[m]; print("n",len(Q),"zmax",Q[:,2].max().round(3))
# body axis from low points only (z<0.93) to avoid the handle
lo=Q[Q[:,2]<0.935]; cc=lo[:,:2].mean(0); U,S,Vt=np.linalg.svd(lo[:,:2]-cc,full_matrices=False); ax=Vt[0]; pe=Vt[1]
print("axis",ax.round(3),"az",np.degrees(np.arctan2(ax[1],ax[0])).round(1),"center_lo",cc.round(3))
a=(Q[:,:2]-cc)@ax; b=(Q[:,:2]-cc)@pe
for s in np.arange(a.min(),a.max(),0.01):
    mm=(a>=s)&(a<s+0.01)
    if mm.sum()<3: continue
    top=Q[mm][Q[mm,2].argmax()]
    print(f" a={s:+.3f} n={mm.sum():4d} zmax={Q[mm,2].max():.3f} at perp={b[mm][Q[mm,2].argmax()]:+.3f}  perp[{b[mm].min():+.3f},{b[mm].max():+.3f}]  perp(z<0.93)[{b[mm&(Q[:,2]<0.93)].min() if (mm&(Q[:,2]<0.93)).any() else np.nan:+.3f},{b[mm&(Q[:,2]<0.93)].max() if (mm&(Q[:,2]<0.93)).any() else np.nan:+.3f}]")
EOF
python3 lying.py hovB_eye_world.npy -0.167 0.277

# openrua op 61
python3 lying.py hovB_eye_world.npy -0.167 0.277 0.906

# openrua op 62
cat > /workspace/hmap.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
cx,cy=float(sys.argv[2]),float(sys.argv[3]); half=float(sys.argv[4]) if len(sys.argv)>4 else 0.13; res=0.0075
zt=float(sys.argv[5]) if len(sys.argv)>5 else 0.905
xs=np.arange(cx-half,cx+half,res); ys=np.arange(cy-half,cy+half,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((P[:,0]-(cx-half))/res).astype(int); iy=((P[:,1]-(cy-half))/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for i,j,z in zip(ix[ok],iy[ok],P[ok,2]):
    if np.isnan(H[i,j]) or z>H[i,j]: H[i,j]=z
# rows = x (increasing downward), cols = y (increasing right); digit = cm above table
print("     y:"+"".join(f"{y:+.2f}"[-3:] if k%4==0 else "   " for k,y in enumerate(ys)))
for i,x in enumerate(xs):
    row=""
    for j in range(len(ys)):
        z=H[i,j]
        if np.isnan(z): row+=" "
        elif z<zt: row+="."
        else:
            d=int(round((z-0.894)*100)); row+=str(min(d,9)) if d<10 else chr(ord('A')+min(d-10,25))
    print(f"x={x:+.3f} {row}")
EOF
python3 hmap.py hovB_eye_world.npy -0.167 0.277

# openrua op 63
cat > /workspace/rows.py <<'EOF'
import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
y0,y1=float(sys.argv[2]),float(sys.argv[3]); x0,x1=float(sys.argv[4]),float(sys.argv[5]); zt=float(sys.argv[6]) if len(sys.argv)>6 else 0.905
m=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,0]>x0)&(P[:,0]<x1)&(P[:,2]>zt)&(P[:,2]<1.1)
Q=P[m]
for x in np.arange(x0,x1,0.004):
    mm=(Q[:,0]>=x)&(Q[:,0]<x+0.004)
    if mm.sum()<3: continue
    R=Q[mm]; hi=R[R[:,2]>R[:,2].max()-0.01]
    print(f"x={x:+.3f} n={mm.sum():4d} y[{R[:,1].min():.3f},{R[:,1].max():.3f}] w={R[:,1].max()-R[:,1].min():.3f} zmax={R[:,2].max():.3f} ytop={hi[:,1].mean():.3f} ")
EOF
python3 rows.py hovB_eye_world.npy 0.20 0.292 -0.27 -0.06

# openrua op 64
python3 hover.py 0.16 0.06 1.22 90 hovA 2>&1 | tail -4 && python3 hmap.py hovA_eye_world.npy 0.197 0.066 0.14 0.94

# openrua op 65
cat > /workspace/iktest.py <<'EOF'
import numpy as np, itertools
from rob import *
r=Rob("iktest")
seed=r.arm_q()
def hR(hz,hy): return R_from_axes(hz,hy)
tests=[]
for x in (0.10,0.13,0.16,0.20):
  for z in (1.00,1.08,1.16,1.24):
    for tilt in (0,20,35):
      th=np.radians(tilt); hz=np.array([np.sin(th),0,-np.cos(th)])
      for hyaz in (90,):
        hy=np.array([0,1.0,0])
        q=r.ik([x,0.05,z],hR(hz,hy),seed=seed,at_tcp=True)
        print(f"x={x} z={z} tilt={tilt}: {'ok '+str(q.round(2)) if q is not None else 'FAIL'}")
EOF
python3 iktest.py 2>&1 | grep -v "^\[" 

# openrua op 66
python3 hmap.py bird_now.npy 0.197 0.066 0.13 0.94

# openrua op 67
cat > /workspace/handgeom.py <<'EOF'
import numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("handgeom"); buf=Buffer(); TransformListener(buf,r.node)
def cloud(cam):
    fr=f"{cam}_optical_frame"
    while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
    t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
    P=r.depth_world(cam,[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w]).reshape(-1,3); return P[np.isfinite(P).all(1)]
p,R=r.fk(); pt=p+TCP*R[:,2]
print("hand origin",p.round(3),"tcp",pt.round(3)); print("R\n",R.round(2))
for cam in ("sideview","agentview","birdview"):
    P=cloud(cam)
    L=(P-pt)@R   # coords in hand frame relative to TCP: columns hx,hy,hz
    m=(np.abs(L[:,0])<0.12)&(np.abs(L[:,1])<0.15)&(L[:,2]>-0.25)&(L[:,2]<0.03)&(P[:,2]>1.0)
    Q=L[m]; print(cam,"n",m.sum())
    for z0 in np.arange(-0.25,0.03,0.01):
        mm=(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)
        if mm.sum()>3: print(f"  hz[{z0:+.2f},{z0+0.01:+.2f}] n={mm.sum():4d} hx[{Q[mm,0].min():+.3f},{Q[mm,0].max():+.3f}] hy[{Q[mm,1].min():+.3f},{Q[mm,1].max():+.3f}]")
EOF
python3 handgeom.py 2>&1 | grep -v "^\["

# openrua op 68
cat > /workspace/iktest2.py <<'EOF'
import numpy as np
from rob import *
r=Rob("iktest2"); seed=r.arm_q()
a=np.array([0,0.966,-0.259])   # pot A axis, lid->base
def tryp(name,p,hz,hy=None):
    hz=np.asarray(hz,float); hz/=np.linalg.norm(hz)
    if hy is None:
        hy=np.cross(a,hz); hy/=np.linalg.norm(hy)
        if hy[0]>0: hy=-hy
    R=R_from_axes(hz,hy); q=r.ik(p,R,seed=seed,at_tcp=True)
    fl=np.asarray(p)-TCP*hz
    print(f"{name}: hz={hz.round(2)} hy={R[:,1].round(2)} flange_r={np.hypot(fl[0]+0.66,fl[1]):.3f} -> {'ok '+str(q.round(2)) if q is not None else 'FAIL'}")
for tx in (0.2,0.3,0.4):
  for ty in (0.0,0.2,0.34):
    hz=np.array([tx,ty,-np.sqrt(max(1e-6,1-tx*tx-ty*ty))])
    tryp(f"lidA tx{tx} ty{ty}",[0.197,0.008,1.0],hz)
# hover above A for wrist cam, tilt 20 toward +x
tryp("hoverA",[0.16,0.03,1.22],[0.34,0,-0.94],[0,1,0])
tryp("hoverA2",[0.14,0.03,1.18],[0.34,0,-0.94],[0,1,0])
EOF
python3 iktest2.py 2>&1 | grep -v "^\["

# openrua op 69
cat > /workspace/hover2.py <<'EOF'
import sys, numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("hover2")
x,y,z,tx,ty,hyaz=map(float,sys.argv[1:7]); tag=sys.argv[7]
hz=np.array([tx,ty,-np.sqrt(1-tx*tx-ty*ty)]); hy=np.array([np.cos(np.radians(hyaz)),np.sin(np.radians(hyaz)),0.0])
R=R_from_axes(hz,hy); q=r.ik([x,y,z],R,at_tcp=True)
if q is None: raise SystemExit("IK FAIL")
dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15); print(r.move_q(q,dur)); p,Rn=r.tcp(); print("tcp",p.round(3),"hz",Rn[:,2].round(2))
r.snap("robot0_eye_in_hand",f"/workspace/{tag}_eye.png")
buf=Buffer(); TransformListener(buf,r.node); fr="robot0_eye_in_hand_optical_frame"
while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
P=r.depth_world("robot0_eye_in_hand",[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w]); np.save(f"/workspace/{tag}_eye_world.npy",P)
print("cam at",np.round([tr.x,tr.y,tr.z],3))
EOF
python3 hover2.py 0.14 0.03 1.18 0.34 0 90 hovA 2>&1 | grep -v "^\[" && python3 hmap.py hovA_eye_world.npy 0.197 0.07 0.13 0.94

# openrua op 70
cat > /workspace/stageA1.py <<'EOF'
"""Pot A stage 1: pinch boiler top-down (tilted 20deg), lift (pot pivots lid-down),
carry to table, lay down lying with lid toward -x, release."""
import sys, numpy as np
from rob import *
r=Rob("stageA1")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
def unit(v): v=np.asarray(v,float); return v/np.linalg.norm(v)
# --- pinch pose on stove
PA=np.array([0.200,0.120]); ZP=0.960
hz_p=unit([0.34,0,-0.94]); hy_p=unit([-0.94,0,-0.34]); R_p=R_from_axes(hz_p,hy_p)
# --- lay-down geometry
L=np.array([-0.29,-0.22,0.894])     # where knob tip meets table
POT=0.138                           # pinch point -> knob tip
def lay(theta_deg, dz=0.0):
    th=np.radians(theta_deg); hz=np.array([-np.sin(th),0,-np.cos(th)])
    P=L-POT*hz+np.array([0,0,dz]); return P, R_from_axes(hz,[0,1,0])
steps=[("pre",   (np.r_[PA,1.10], R_p)),
       ("pinch", (np.r_[PA,ZP],  R_p)),
       ("lift",  (np.r_[PA,1.25], R_p)),
       ("mid",   (np.array([0.0,-0.05,1.25]), R_from_axes([0,0,-1],[-0.7,0.7,0]))),
       ("hang",  lay(0,0.20)),
       ("lay0",  lay(0,0.013)),
       ("lay25", lay(25)),("lay50", lay(50)),("lay65", lay(65)),("lay75", lay(75)),
       ("free",  lay(75,0.12))]
sols={}; seed=r.arm_q()
for name,(p,R) in steps:
    q=r.ik(p,R,seed=seed,at_tcp=True)
    if q is None: print(name,"IK FAIL at",p.round(3)); sys.exit(1)
    sols[name]=q; print(f"{name}: p={p.round(3)} hz={R[:,2].round(2)} q={q.round(2)} dq={np.abs(q-seed).max():.2f}"); seed=q
if not EXEC: sys.exit(0)
def go(name):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>0.03: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04)); 
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("pinch")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.015<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift")
f=np.abs(r.fingers()); print("fingers after lift:",f,flush=True)
if min(f)<0.015: raise SystemExit("DROPPED")
for cam in ("agentview","sideview"): r.snap(cam,f"/workspace/A1_lift_{cam}.png")
go("mid"); go("hang")
r.snap("agentview","/workspace/A1_hang_agentview.png"); r.snap("sideview","/workspace/A1_hang_sideview.png")
for s in ("lay0","lay25","lay50","lay65","lay75"): go(s)
r.gripper(0.04)
go("free")
for cam in ("agentview","sideview","birdview"): r.snap(cam,f"/workspace/A1_done_{cam}.png")
print("DONE",flush=True)
EOF
python3 stageA1.py 2>&1 | grep -v "^\["

# openrua op 71
python3 - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np
from rob import *
r=Rob("t"); seed=r.arm_q()
def unit(v): v=np.asarray(v,float); return v/np.linalg.norm(v)
for tilt in (30,35,40,45):
    t=np.radians(tilt); hz=np.array([np.sin(t),0,-np.cos(t)]); hy=np.array([-np.cos(t),0,-np.sin(t)])
    C=np.array([0.2,0.12,0.96]); T=C-0.008*hz
    for z in (T[2],T[2]+0.01,1.10,1.25):
        p=np.array([T[0],T[1],z]); q=r.ik(p,R_from_axes(hz,hy),seed=seed,at_tcp=True)
        print(f"tilt={tilt} p={p.round(3)}: {'ok '+str(q.round(2)) if q is not None else 'FAIL'}")
EOF

# openrua op 72
python3 - <<'EOF'
import re
s=open("stageA1.py").read()
s=s.replace("""PA=np.array([0.200,0.120]); ZP=0.960
hz_p=unit([0.34,0,-0.94]); hy_p=unit([-0.94,0,-0.34]); R_p=R_from_axes(hz_p,hy_p)""",
"""TILT=np.radians(35)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); hy_p=np.array([-np.cos(TILT),0,-np.sin(TILT)]); R_p=R_from_axes(hz_p,hy_p)
C=np.array([0.200,0.120,0.960]); T=C-0.008*hz_p      # pads a bit behind the boiler axis: keeps fingertips off the plate
PA=T[:2]; ZP=T[2]""")
s=s.replace("""sols={}; seed=r.arm_q()
for name,(p,R) in steps:
    q=r.ik(p,R,seed=seed,at_tcp=True)
    if q is None: print(name,"IK FAIL at",p.round(3)); sys.exit(1)
    sols[name]=q; print(f"{name}: p={p.round(3)} hz={R[:,2].round(2)} q={q.round(2)} dq={np.abs(q-seed).max():.2f}"); seed=q""",
"""sols={}
# solve the pinch first (hardest), then chain outward from it
order=["pinch","pre","lift","mid","hang","lay0","lay25","lay50","lay65","lay75","free"]
sd=dict(steps); seed=None
for name in order:
    p,R=sd[name]
    q=None
    for s0 in ([seed] if seed is not None else [])+[r.arm_q(),np.array([0.5,1.35,-0.59,-0.98,0.84,2.76,2.16]),np.zeros(7)]:
        q=r.ik(p,R,seed=s0,at_tcp=True)
        if q is not None: break
    if q is None: print(name,"IK FAIL at",p.round(3)); sys.exit(1)
    zs={l:r.fk(q,l)[0][2] for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7"]}
    sols[name]=q; print(f"{name}: p={p.round(3)} hz={R[:,2].round(2)} q={q.round(2)} dq={np.abs(q-seed).max() if seed is not None else 0:.2f} minlinkz={min(zs.values()):.3f}"); seed=q
# sanity: consecutive steps in execution order should be on the same branch
prev=None
for name,_ in steps:
    if prev: print(f"  {prev}->{name}: max dq={np.abs(sols[name]-sols[prev]).max():.2f}")
    prev=name""")
open("stageA1.py","w").write(s)
EOF
python3 stageA1.py 2>&1 | grep -v "^\["

# openrua op 73
cat > /workspace/stageA1.py <<'EOF'
"""Pot A stage 1a: pinch boiler top-down (tilted 35deg), lift (pot pivots lid-down)."""
import sys, numpy as np
from rob import *
r=Rob("stageA1")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
TILT=np.radians(35)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); hy_p=np.array([-np.cos(TILT),0,-np.sin(TILT)]); R_p=R_from_axes(hz_p,hy_p)
C=np.array([0.200,0.120,0.960]); T=C-0.008*hz_p      # pads a bit behind the boiler axis: keeps fingertips off the plate
steps=[("pre",   (np.r_[T[:2],1.10], R_p)),
       ("pinch", (T,  R_p)),
       ("lift",  (np.r_[T[:2],1.25], R_p))]
FWD=np.array([0.5,1.35,-0.59,-0.98,0.84,2.76,2.16])
sols={}; sd=dict(steps)
for name,seed in (("pinch",FWD),("pre",None),("lift",None)):
    p,R=sd[name]; q=r.ik(p,R,seed=seed if seed is not None else sols["pinch"],at_tcp=True)
    if q is None: print(name,"IK FAIL"); sys.exit(1)
    zs={l:r.fk(q,l)[0][2] for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7"]}
    sols[name]=q; print(f"{name}: p={p.round(3)} q={q.round(2)} minlinkz={min(zs.values()):.3f}")
print("pre->pinch dq",np.abs(sols["pre"]-sols["pinch"]).max().round(2)," pinch->lift dq",np.abs(sols["lift"]-sols["pinch"]).max().round(2))
np.save("solsA1.npy",sols)
if not EXEC: sys.exit(0)
def go(name):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>0.03: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04))
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("pinch")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.015<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift")
f=np.abs(r.fingers()); print("fingers after lift:",f,flush=True)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/A1_lift_{cam}.png")
print("DONE",flush=True)
EOF
python3 stageA1.py 2>&1 | grep -v "^\["

# openrua op 74
python3 -u stageA1.py go > A1a.log 2>&1 &
sleep 100; cat A1a.log | grep -v "^\[move_q\]"

# openrua op 75
sleep 60; cat A1a.log | grep -v "^\[move_q\]"

# openrua op 76
python3 -c "
from rob import *
r=Rob('s'); r.snap('agentview','/workspace/stuck_agent.png'); r.snap('sideview','/workspace/stuck_side.png'); print(r.tcp()[0].round(3), r.wrench()[:3].round(1))"

# openrua op 77
cat stageA1.py && cat hmap.py | head -30

# openrua op 78
sed -n 1,60p hover2.py

# openrua op 79
cat > /workspace/eyecloud.py <<'EOF'
"""Move to a saved joint solution (optional) and dump wrist-cam depth as world cloud."""
import sys, numpy as np, rclpy
from rob import *
from tf2_ros import Buffer, TransformListener
r=Rob("eyecloud"); tag=sys.argv[1]
if len(sys.argv)>3:
    sols=np.load(sys.argv[2],allow_pickle=True).item(); q=sols[sys.argv[3]]
    dur=max(3.0,np.abs(q-r.arm_q()).max()/0.15); print(r.move_q(q,dur))
p,Rn=r.tcp(); print("tcp",p.round(3),"hz",Rn[:,2].round(2),"F",r.wrench()[:3].round(1))
r.snap("robot0_eye_in_hand",f"/workspace/{tag}_eye.png")
buf=Buffer(); TransformListener(buf,r.node); fr="robot0_eye_in_hand_optical_frame"
while not buf.can_transform("world",fr,rclpy.time.Time()): r.spin(0.2)
t=buf.lookup_transform("world",fr,rclpy.time.Time()); tr=t.transform.translation; ro=t.transform.rotation
P=r.depth_world("robot0_eye_in_hand",[tr.x,tr.y,tr.z],[ro.x,ro.y,ro.z,ro.w]); np.save(f"/workspace/{tag}_eye_world.npy",P)
print("cam at",np.round([tr.x,tr.y,tr.z],3))
EOF
python3 -u eyecloud.py A1pre solsA1.npy pre > eye.log 2>&1; cat eye.log; python3 hmap.py A1pre_eye_world.npy 0.2 0.08 0.1 0.93

# openrua op 80
python3 -c "
import numpy as np
P=np.load('A1pre_eye_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for y0 in (0.10,0.11,0.12,0.13):
    m=(abs(P[:,1]-y0)<0.005)&(P[:,2]>0.94)&(P[:,0]>0.15)&(P[:,0]<0.26)
    Q=P[m]; 
    if len(Q)<5: print(y0,'none'); continue
    # circle fit x,z
    A=np.c_[2*Q[:,0],2*Q[:,2],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,2]**2
    cx,cz,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx**2+cz**2)
    print(f'y={y0}: n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] zmax={Q[:,2].max():.3f} circle c=({cx:.3f},{cz:.3f}) r={r:.3f}')
"


# openrua op 81
python3 -c "
import numpy as np
P=np.load('A1pre_eye_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for y0 in np.arange(0.06,0.15,0.01):
    m=(abs(P[:,1]-y0)<0.005)&(P[:,2]>0.94)&(P[:,2]<1.02)&(P[:,0]>0.16)&(P[:,0]<0.26)
    Q=P[m]
    if len(Q)<5: print(y0,'none'); continue
    A=np.c_[2*Q[:,0],2*Q[:,2],np.ones(len(Q))]; b=Q[:,0]**2+Q[:,2]**2
    cx,cz,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx**2+cz**2)
    print(f'y={y0:.2f}: n={len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] zmax={Q[:,2].max():.3f} circle c=({cx:.3f},{cz:.3f}) r={r:.3f}')
"


# openrua op 82
cat > /workspace/stageA1.py <<'EOF'
"""Pot A stage 1a: pinch boiler top-down (tilted), lift (pot pivots lid-down)."""
import sys, numpy as np
from rob import *
r=Rob("stageA1")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
TILT=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 25)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); hy_p=np.array([-np.cos(TILT),0,-np.sin(TILT)]); R_p=R_from_axes(hz_p,hy_p)
# boiler axis measured from wrist depth: x=0.2115, z=0.972 at y=0.108 (axial ~4cm from base, r~3.4)
C=np.array([0.2115,0.108,0.972]); T=C-0.006*hz_p
steps=[("pre",   (np.r_[T[:2],1.10], R_p)),
       ("pinch", (T,  R_p)),
       ("lift",  (np.r_[T[:2],1.25], R_p))]
FWD=np.array([0.5,1.35,-0.59,-0.98,0.84,2.76,2.16])
sols={}; sd=dict(steps)
for name,seed in (("pinch",FWD),("pre",None),("lift",None)):
    p,R=sd[name]; q=r.ik(p,R,seed=seed if seed is not None else sols["pinch"],at_tcp=True)
    if q is None: print(name,"IK FAIL"); sys.exit(1)
    zs={l:r.fk(q,l)[0][2] for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7"]}
    sols[name]=q; print(f"{name}: p={p.round(3)} q={q.round(2)} minlinkz={min(zs.values()):.3f}")
print("pre->pinch dq",np.abs(sols["pre"]-sols["pinch"]).max().round(2)," pinch->lift dq",np.abs(sols["lift"]-sols["pinch"]).max().round(2))
np.save("solsA1.npy",sols)
if not EXEC: sys.exit(0)
def go(name):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>0.03: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04))
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("pinch")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.015<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift")
f=np.abs(r.fingers()); print("fingers after lift:",f,flush=True)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/A1_lift_{cam}.png")
print("DONE",flush=True)
EOF
for t in 20 25 30; do echo "--- tilt $t"; python3 stageA1.py plan $t 2>&1 | grep -v move_q; done

# openrua op 83
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rp(t):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); hy=np.array([-np.cos(t),0,-np.sin(t)]); return R_from_axes(hz,hy)
FWD=np.array([0.35,1.48,-0.66,-0.59,0.72,2.37,2.33])
for t in (25,30):
    for z in (1.18,1.20,1.22,1.25):
        q=r.ik([0.209,0.108,z],Rp(t),seed=FWD,at_tcp=True)
        print(t,z,None if q is None else q.round(2))
EOF

# openrua op 84
sed -i 's/("lift",  (np.r_\[T\[:2\],1.25\], R_p))/("lift",  (np.r_[T[:2],1.20], R_p))/' stageA1.py && grep -n lift stageA1.py | head -3 && python3 -u stageA1.py go 25 > A1a.log 2>&1; cat A1a.log | grep -v move_q

# openrua op 85
sed -i 's/^C=np.array(\[0.2115,0.108,0.972\]); T=C-0.006\*hz_p/C=np.array([0.2115,0.108,0.975]); T=C.copy()/' stageA1.py && grep -n "^C=" stageA1.py && python3 -u stageA1.py go 25 > A1a.log 2>&1; cat A1a.log | grep -v move_q

# openrua op 86
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rp(t):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); hy=np.array([-np.cos(t),0,-np.sin(t)]); return R_from_axes(hz,hy)
S=np.array([0.35,1.48,-0.66,-0.59,0.72,2.37,2.33])
for t in (25,27,30):
    for z in (0.975,0.98):
        q=r.ik([0.2115,0.108,z],Rp(t),seed=S,at_tcp=True)
        print(t,z,None if q is None else q.round(2))
EOF

# openrua op 87
sed -i 's/^FWD=np.array(\[0.5,1.35,-0.59,-0.98,0.84,2.76,2.16\])/FWD=np.array([0.35,1.48,-0.66,-0.59,0.72,2.37,2.33])/' stageA1.py && python3 -u stageA1.py go 27 > A1a.log 2>&1; cat A1a.log | grep -v move_q

# openrua op 88
python3 -c "
from rob import *
r=Rob('s'); r.snap('agentview','/workspace/miss_agent.png'); r.snap('sideview','/workspace/miss_side.png'); r.snap('robot0_eye_in_hand','/workspace/miss_eye.png')"

# openrua op 89
python3 -u park.py 2 > park.log 2>&1; tail -2 park.log; python3 -c "
from rob import *
import numpy as np
r=Rob('s'); P=r.depth_world('birdview',*BIRD); np.save('bird_now.npy',P); r.snap('birdview','/workspace/now_birdview.png'); r.snap('agentview','/workspace/now_agentview.png')" && python3 hmap.py bird_now.npy 0.28 0.05 0.16 0.90

# openrua op 90
python3 -u hover2.py 0.17 0.02 1.18 0.707 0 90 hovA2 > hov.log 2>&1; grep -v move_q hov.log; python3 hmap.py hovA2_eye_world.npy 0.29 0.04 0.13 0.90

# openrua op 91
python3 -c "
import numpy as np
P=np.load('hovA2_eye_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>0.23)&(P[:,0]<0.275)&(abs(P[:,1]-0.015)<0.05)&(P[:,2]>0.94)
Q=P[m]
for z0 in np.arange(0.94,1.01,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f'z {z0:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
# the visible -x face of the handle: for each y bin the min x at z>0.96
for y0 in np.arange(-0.02,0.06,0.01):
    s=Q[(abs(Q[:,1]-y0)<0.005)&(Q[:,2]>0.96)]
    if len(s): print(f'y {y0:+.2f}: xmin={s[:,0].min():.3f} at z={s[s[:,0].argmin(),2]:.3f}  zmax={s[:,2].max():.3f} at x={s[s[:,2].argmax(),0]:.3f}')
"


# openrua op 92
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,1,0])
seeds=[np.array([0.35,1.48,-0.66,-0.59,0.72,2.37,2.33]), np.array([0.5,1.35,-0.59,-0.98,0.84,2.76,2.16]), None]
for t in (35,45,55):
    for z in (0.966,1.12):
        for s in seeds:
            q=r.ik([0.259,0.028,z],Rh(t),seed=s,at_tcp=True)
            if q is not None: break
        print(t,z,None if q is None else q.round(2))
EOF

# openrua op 93
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
rng=np.random.default_rng(0)
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def Rh(t,sgn=1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
for t in (45,55,65,75):
  for sgn in (1,-1):
    found=None
    for k in range(12):
        s=rng.uniform(lim[:,0],lim[:,1])
        q=r.ik([0.259,0.028,0.966],Rh(t,sgn),seed=s,at_tcp=True)
        if q is not None: found=q; break
    print(t,sgn,None if found is None else found.round(2))
EOF

# openrua op 94
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
base=np.array([0.37,1.47,-0.66,-0.63,0.74,2.43,2.31])
for d in (-1.57,1.57):
    s=base.copy(); s[6]+=d
    q=r.ik([0.212,0.108,0.975],Rh(27,1),seed=s,at_tcp=True); print("boiler spot hy=+y q7seed",round(s[6],2),None if q is None else q.round(2))
    q=r.ik([0.212,0.108,0.975],Rh(27,-1),seed=s,at_tcp=True); print("boiler spot hy=-y q7seed",round(s[6],2),None if q is None else q.round(2))
# FK check: does q7 rotation only change orientation?
p1,R1=r.tcp(base); s=base.copy(); s[6]-=1.57; p2,R2=r.tcp(s); print("tcp shift",(p2-p1).round(4), "hy1",R1[:,1].round(2),"hy2",R2[:,1].round(2))
EOF

# openrua op 95
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
S=np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])
for t in (30,35,40,45,50,55,60):
    for x in (0.245,0.259):
        q=r.ik([x,0.028,0.966],Rh(t,-1),seed=S,at_tcp=True)
        print(t,x,None if q is None else q.round(2))
EOF

# openrua op 96
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
S=np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])
for t in (27,35,45,55):
    row=[]
    for x in (0.21,0.22,0.23,0.24):
        q=r.ik([x,0.028,0.966],Rh(t,-1),seed=S,at_tcp=True)
        row.append("ok" if q is not None else "--")
    print(t,row)
EOF

# openrua op 97
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=-1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
S=np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])
last=S
for z in (0.95,0.97,1.0,1.03):
  for t in (60,70,80,90):
    row=[]
    for x in (0.24,0.25,0.26,0.27,0.28,0.29,0.30):
        q=None
        for s in (last,S):
            q=r.ik([x,0.03,z],Rh(t),seed=s,at_tcp=True)
            if q is not None: last=q; break
        row.append("ok" if q is not None else "--")
    print(z,t,row)
EOF

# openrua op 98
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
q=r.arm_q(); print("q",q.round(2))
for l in ["panda_link0","panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]:
    try: p,R=r.fk(q,l); print(l,p.round(3))
    except Exception as e: print(l,e)
# straight-arm test: q = [0, 1.5, 0, -0.07, 0, 1.57, 0]? compute FK of nearly-straight configurations
for qq in ([0,1.2,0,-0.1,0,1.3,0.785],[0,1.4,0,-0.1,0,1.5,0.785],[0,1.6,0,-0.1,0,1.7,0.785]):
    p,R=r.tcp(np.array(qq)); print(qq,"tcp",p.round(3),"hz",R[:,2].round(2))
EOF

# openrua op 99
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
best=[]
for q2 in np.arange(1.2,1.77,0.08):
  for q4 in (-0.07,-0.3):
    for q6 in np.arange(0.0,3.76,0.2):
        q=np.array([0.03,q2,0,q4,0,q6,0.785-1.57])
        p,R=r.tcp(q)
        if 0.94<p[2]<1.03 and p[0]>0.24: best.append((round(p[0],3),round(p[2],3),R[:,2].round(2).tolist(),q.round(2).tolist()))
for b in sorted(best,key=lambda b:-b[0])[:25]: print(b)
EOF

# openrua op 100
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=-1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
seeds=[np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75]),np.array([0.03,1.68,0,-0.3,0,3.0,-0.78]),np.array([0.03,1.6,0,-0.3,0,2.6,-0.78])]
last=None
res=[]
for t in (55,60,65,70,75,80):
  for z in (0.98,0.99,1.0):
    for x in (0.25,0.255,0.26,0.265):
        q=None
        for s in ([last] if last is not None else [])+seeds:
            q=r.ik([x,0.028,z],Rh(t),seed=s,at_tcp=True)
            if q is not None: last=q; break
        if q is not None:
            # pad face region coverage: x range at handle height
            th=np.radians(t); hx=np.array([np.cos(th),0,np.sin(th)]); hz=np.array([np.sin(th),0,-np.cos(th)])
            pts=np.array([[x,0,z]+a*hx+b*hz for a in (-0.01,0.01) for b in (-0.04,0.005)])
            res.append((t,x,z,q.round(2).tolist(),"face x[%.3f,%.3f] z[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,2].min(),pts[:,2].max())))
for rr in res: print(rr)
EOF

# openrua op 101
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
def Rh(t,sgn=-1):
    t=np.radians(t); hz=np.array([np.sin(t),0,-np.cos(t)]); return R_from_axes(hz,[0,sgn,0])
seeds=[np.array([0.16,1.65,-0.42,-0.43,0.63,2.99,0.61]),np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75]),np.array([0.09,1.48,-0.12,-0.68,0.18,3.11,0.75])]
last=None
for t in (40,45,50,55):
  for z in (0.966,0.972,0.978):
    row=[]
    for x in (0.24,0.243,0.246,0.249,0.252):
        q=None
        for s in ([last] if last is not None else [])+seeds:
            q=r.ik([x,0.012,z],Rh(t),seed=s,at_tcp=True)
            if q is not None: last=q; break
        row.append(("ok "+str(q.round(2).tolist())) if q is not None else "--")
    print(t,z,row)
EOF

# openrua op 102
cat > /workspace/stageA1h.py <<'EOF'
"""Pot A (fallen beside stove, x~0.27-0.35): pinch the handle tip (x~0.245-0.256, z~0.975-0.99)
lengthwise in y with a 50deg-tilted hand at the reach frontier, then lift so the pot hangs from its handle."""
import sys, numpy as np
from rob import *
r=Rob("stageA1h")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
TILT=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 50)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); R_p=R_from_axes(hz_p,[0,-1,0])
X,Y=0.246,0.0125
steps=[("pre",  np.array([X,Y,1.08])),("mid",np.array([X,Y,1.02])),("pinch",np.array([X,Y,0.98])),
       ("lift1",np.array([X,Y,1.05])),("lift2",np.array([0.16,Y,1.15]))]
seeds=[np.array([-0.08,1.51,0.17,-0.66,-0.48,3.03,1.09]),np.array([0.03,1.5,-0.04,-0.7,0.03,3.16,0.8]),
       np.array([0.16,1.65,-0.42,-0.43,0.63,2.99,0.61]),np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])]
sols={}
order=["pinch","mid","pre","lift1","lift2"]; sd=dict(steps)
for name in order:
    q=None
    for s in ([sols["pinch"]] if "pinch" in sols else [])+seeds:
        q=r.ik(sd[name],R_p,seed=s,at_tcp=True)
        if q is not None: break
    if q is None: print(name,"IK FAIL"); sys.exit(1)
    sols[name]=q; print(f"{name}: p={sd[name].round(3)} q={q.round(2)}")
for a,b in zip(order[:-1],order[1:]): print(f"dq {a}->{b}: {np.abs(sols[a]-sols[b]).max():.2f}")
np.save("solsA1h.npy",sols)
if not EXEC: sys.exit(0)
def go(name,tol=0.02):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04))
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("mid"); go("pinch")
r.snap("robot0_eye_in_hand","/workspace/A1h_pinch_eye.png")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.007<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift1")
f=np.abs(r.fingers()); print("fingers after lift1:",f,flush=True)
if min(f)<0.006: raise SystemExit("DROPPED")
go("lift2")
f=np.abs(r.fingers()); print("fingers after lift2:",f,flush=True)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/A1h_lift_{cam}.png")
print("DONE",flush=True)
EOF
python3 stageA1h.py plan 50 2>&1 | grep -v move_q

# openrua op 103
python3 -u stageA1h.py go 50 > A1h.log 2>&1; grep -v move_q A1h.log

# openrua op 104
python3 -c "
from rob import *
r=Rob('s'); r.snap('agentview','/workspace/drop_agent.png'); r.snap('sideview','/workspace/drop_side.png'); r.snap('robot0_eye_in_hand','/workspace/drop_eye.png')"

# openrua op 105
python3 -u park.py 2 > park.log 2>&1; python3 -c "
from rob import *
import numpy as np
r=Rob('s'); P=r.depth_world('birdview',*BIRD); np.save('bird_now.npy',P); r.snap('birdview','/workspace/now_birdview.png')" && python3 hmap.py bird_now.npy 0.29 0.04 0.11 0.90

# openrua op 106
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
seeds=[np.array([-0.08,1.51,0.19,-0.62,-0.47,2.99,1.06]),np.array([0.03,1.5,-0.04,-0.7,0.03,3.16,0.8]),np.array([0.16,1.65,-0.42,-0.43,0.63,2.99,0.61]),np.array([0.03,1.68,0,-0.3,0,3.0,-0.78])]
H=np.array([(0.233,0.96),(0.25,0.96),(0.25,0.975),(0.233,0.975)])
last=None
for t in (50,60,70,80,90):
    th=np.radians(t); hz=np.array([np.sin(th),0,-np.cos(th)]); R=R_from_axes(hz,[0,-1,0]); hx=R[:,0]
    for X in (0.238,0.242,0.246,0.25):
        for Z in (0.962,0.968,0.974):
            q=None
            for s in ([last] if last is not None else [])+seeds:
                q=r.ik([X,0.012,Z],R,seed=s,at_tcp=True)
                if q is not None: last=q; break
            if q is None: continue
            d=H-np.array([X,Z]); a=d[:,0]*hx[0]+d[:,1]*hx[2]; b=d[:,0]*hz[0]+d[:,1]*hz[2]
            marg=min(0.01-a.max(), a.min()+0.01, 0.005-b.max(), b.min()+0.04)
            print(f"t={t} X={X} Z={Z} a[{a.min():+.3f},{a.max():+.3f}] b[{b.min():+.3f},{b.max():+.3f}] margin={marg*1000:.1f}mm q={q.round(2)}")
EOF

# openrua op 107
python3 - <<'EOF' 2>&1 | grep -v move_q
import numpy as np
from rob import *
r=Rob("t")
for q in ([-0.16,1.54,0.45,-0.61,2.88,2.61,-2.58],[-0.08,1.57,0.2,-0.56,-0.47,2.99,1.05]):
    q=np.array(q); print("q",q)
    for l in ["panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]:
        p,R=r.fk(q,l); print("  ",l,p.round(3))
    p,R=r.tcp(q); print("   tcp",p.round(3),"hz",R[:,2].round(2),"hy",R[:,1].round(2),"hx",R[:,0].round(2))
EOF

# openrua op 108
cat > /workspace/dragA.py <<'EOF'
"""Pot A recovery: clamp the handle tip (hy=-y, hand tilted TILT toward +x) and drag the pot along the table.
usage: dragA.py pinch X Y Z | to X Y Z | open | grip | snap tag"""
import sys, numpy as np
from rob import *
r=Rob("dragA")
TILT=np.radians(50)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); R_p=R_from_axes(hz_p,[0,-1,0])
SEEDS=[np.array([-0.08,1.53,0.19,-0.63,-0.49,3.01,1.08]),np.array([-0.08,1.51,0.19,-0.62,-0.47,2.99,1.06]),np.array([0.03,1.5,-0.04,-0.7,0.03,3.16,0.8])]
def solve(p):
    for s in [r.arm_q()]+SEEDS:
        q=r.ik(p,R_p,seed=s,at_tcp=True)
        if q is not None: return q
    raise SystemExit(f"IK FAIL {p}")
def go(p,tol=0.02,tag=""):
    q=solve(p); dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); pp,_=r.tcp()
    print(f"== {tag}{np.round(p,3)}: code={code} jerr={err:.4f} tcp={pp.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
cmd=sys.argv[1]
if cmd=="pinch":
    X,Y,Z=map(float,sys.argv[2:5])
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go([X,Y,1.06],tag="pre"); go([X,Y,Z+0.04],tag="mid"); go([X,Y,Z],tag="pinch")
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.006<min(f)<0.036): raise SystemExit("PINCH MISSED")
elif cmd=="to":
    X,Y,Z=map(float,sys.argv[2:5]); go([X,Y,Z],tag="to")
    f=np.abs(r.fingers())
    if min(f)<0.005: print("!! SLIPPED (fingers closed)")
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="grip":
    print(r.gripper(0.0))
elif cmd=="snap":
    for cam in ("agentview","sideview","robot0_eye_in_hand"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
print("OK",flush=True)
EOF
python3 -u dragA.py pinch 0.242 0.0115 0.966 > drag.log 2>&1; grep -v move_q drag.log

# openrua op 109
for y in -0.03 -0.07 -0.10; do python3 -u dragA.py to 0.242 $y 0.966 2>&1 | grep -v move_q; done

# openrua op 110
python3 -u dragA.py snap slip1 2>&1 | tail -1

# openrua op 111
python3 -u dragA.py open 2>&1 | tail -1; python3 -u dragA.py to 0.2 0.0 1.1 2>&1 | grep -v move_q; python3 -u park.py 2 > park.log 2>&1; python3 -c "
from rob import *
import numpy as np
r=Rob('s'); P=r.depth_world('birdview',*BIRD); np.save('bird_now.npy',P); r.snap('birdview','/workspace/now_birdview.png'); r.snap('agentview','/workspace/now_agentview.png')" && python3 hmap.py bird_now.npy 0.29 0.04 0.11 0.90

# openrua op 112
python3 -c "
from rob import *
r=Rob('s'); r.snap('sideview','/workspace/now_sideview.png'); r.snap('frontview','/workspace/now_frontview.png')"

# openrua op 113
python3 -c "
import cv2
im=cv2.imread('/workspace/now_sideview.png'); h,w=im.shape[:2]
crop=im[250:370,80:260]; cv2.imwrite('/workspace/side_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('/workspace/now_frontview.png'); crop=im[380:480,260:420]; cv2.imwrite('/workspace/front_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 114
mkdir -p "$(dirname /workspace/sweep.py)"
cat > /workspace/sweep.py <<'OPENRUA_EOF'
"""Hand-body sweep of pot A's handle: hz leaning back (toward -x) by t deg, hy=+x
so the hand body's +hy end (0.10 from TCP) reaches over the handle at x~0.30.
  python3 sweep.py test                 -> IK feasibility grid
  python3 sweep.py go X Y Z T [dur]     -> move TCP there (fingers closed first)
  python3 sweep.py pull X Y Z T         -> same as go (used for the -x drag)
  python3 sweep.py snap TAG
"""
import sys, numpy as np
from rob import *
r=Rob("sweep")
def Rlean(t):
    t=np.radians(t); hz=np.array([-np.sin(t),0,-np.cos(t)]); hy=np.array([np.cos(t),0,-np.sin(t)])
    return R_from_axes(hz,hy)
SEEDS=[[0.0,-0.8,0.0,-2.5,0.0,1.7,0.785],[0.36,1.48,-0.66,-0.61,0.74,2.43,0.75],
       [-0.08,1.51,0.17,-0.66,-0.48,3.03,1.09],[0.0,1.2,0.0,-1.2,0.0,2.4,0.785],
       [0.0,1.5,0.0,-0.8,0.0,2.3,0.785],[0.0,1.6,0.0,-0.5,0.0,2.1,-0.785]]
def ik(p,R):
    for s in [r.arm_q()]+[np.array(x) for x in SEEDS]:
        q=r.ik(p,R,seed=s,at_tcp=True)
        if q is not None: return q
    return None
def hand_end(p,R):  # +hy end of hand body, at hz=-0.045 and -0.11 from TCP
    return [(p+0.10*R[:,1]+d*R[:,2]).round(3) for d in (-0.045,-0.11)]
cmd=sys.argv[1]
if cmd=="test":
    for t in (0,10,20,30):
        for x in (0.17,0.19,0.21):
            for z in (0.94,0.96):
                R=Rlean(t); p=np.array([x,-0.05,z]); q=ik(p,R)
                if q is None: print(f"t={t} x={x} z={z}: FAIL"); continue
                zs={l:r.fk(q,l)[0].round(3) for l in ["panda_link6","panda_link7"]}
                print(f"t={t} x={x} z={z}: q={q.round(2)} end={hand_end(p,R)} l7={zs['panda_link7']}")
elif cmd in ("go","pull"):
    x,y,z,t=map(float,sys.argv[2:6]); dur=float(sys.argv[6]) if len(sys.argv)>6 else None
    R=Rlean(t); p=np.array([x,y,z]); q=ik(p,R)
    if q is None: raise SystemExit("IK FAIL")
    d=dur or max(2.5,np.abs(q-r.arm_q()).max()/0.12)
    code,err=r.move_q(q,d); pt,Rn=r.tcp()
    print(f"{cmd}: code={code} err={err:.4f} tcp={pt.round(3)} hz={Rn[:,2].round(2)} hy={Rn[:,1].round(2)} F={r.wrench()[:3].round(1)} fingers={np.round(np.abs(r.fingers()),4)}",flush=True)
elif cmd=="grip":
    print(r.gripper(0.0))
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="snap":
    for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
    print("snapped")
OPENRUA_EOF

# openrua op 115
timeout 600 python3 sweep.py test 2>&1 | grep -v "^\[" | tail -30

# openrua op 116
cat > sweeptest2.py <<'EOF'
import numpy as np
from sweep import *
for sgn in (1,-1):
    for t in (0,15,30):
        for x in (0.10,0.14,0.18):
            t_=np.radians(t); hz=np.array([-np.sin(t_),0,-np.cos(t_)]); hy=sgn*np.array([np.cos(t_),0,-np.sin(t_)])
            R=R_from_axes(hz,hy); p=np.array([x,-0.05,0.96]); q=ik(p,R)
            print(f"sgn={sgn} t={t} x={x}: {'FAIL' if q is None else q.round(2)}",flush=True)
EOF
sed -i 's/^cmd=sys.argv\[1\]/cmd=sys.argv[1] if len(sys.argv)>1 else ""/' sweep.py
timeout 900 python3 sweeptest2.py 2>&1 | grep -v "^\[" | tail -20

# openrua op 117
mkdir -p "$(dirname /workspace/reach.py)"
cat > /workspace/reach.py <<'OPENRUA_EOF'
"""Brute-force reach frontier of the TCP (and fingertip) via local Panda FK."""
import numpy as np, itertools, sys
BASE=np.array([-0.66,0,0.912])
DH=[(0,0.333,0),( -np.pi/2,0,0),(np.pi/2,0.316,0),(np.pi/2,0,0.0825),(-np.pi/2,0.384,-0.0825),(np.pi/2,0,0),(np.pi/2,0,0.088)]
def T(alpha,d,a,th):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    M=np.eye(4)
    for (al,d,a),th in zip(DH,q): M=M@T(al,d,a,th)
    M=M@T(0,0.107,0,0)          # flange
    M=M@T(0,0,0,-np.pi/4)       # hand
    p=M[:3,3]+BASE; R=M[:3,:3]
    return p+0.1034*R[:,2], R
if __name__=="__main__":
    if sys.argv[1]=="check":
        q=np.array([0,-0.8,0,-2.5,0,1.7,0.785]); p,R=fk(q); print("park tcp",p.round(4),"hz",R[:,2].round(3))
        q=np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75]); p,R=fk(q); print("seed tcp",p.round(4),"hz",R[:,2].round(3))
    else:
        zlo,zhi=float(sys.argv[1]),float(sys.argv[2])
        best=[]
        for q2 in np.linspace(0.8,1.76,25):
            for q3 in np.linspace(-0.6,0.6,5):
                for q4 in np.linspace(-1.6,-0.07,32):
                    for q5 in np.linspace(-0.8,0.8,5):
                        for q6 in np.linspace(0.0,3.75,38):
                            q=np.array([0,q2,q3,q4,q5,q6,0.785]); p,R=fk(q)
                            if zlo<=p[2]<=zhi and abs(p[1])<0.06:
                                tip=p+0.005*R[:,2]
                                best.append((p[0],tip[0],q2,q3,q4,q5,q6,p[1],p[2],*R[:,2].round(2)))
        best.sort(reverse=True)
        for b in best[:15]: print(np.round(b,3))
OPENRUA_EOF

# openrua op 118
python3 reach.py check && timeout 900 python3 -c "
import numpy as np
from rob import *
r=Rob('chk')
for q in ([0,-0.8,0,-2.5,0,1.7,0.785],[0.36,1.48,-0.66,-0.61,0.74,2.43,0.75]):
    p,R=r.tcp(np.array(q)); print('svc',p.round(4),R[:,2].round(3))
" 2>&1 | grep -v "^\["

# openrua op 119
time python3 reach.py 0.94 1.0 2>&1 | tail -16

# openrua op 120
python3 reach.py 1.0 1.06 2>&1 | head -4; echo ---; python3 reach.py 1.06 1.15 2>&1 | head -4

# openrua op 121
timeout 600 python3 hover2.py 0.17 -0.02 1.18 0.707 0 90 hovA3 2>&1 | grep -v "^\[" | tail -3 && python3 hmap.py hovA3_eye_world.npy 0.28 0.0 0.11 0.90 2>&1 | tail -40

# openrua op 122
timeout 300 python3 -c "
import numpy as np
from rob import *
r=Rob('bird'); P=r.depth_world('birdview',*BIRD); np.save('/workspace/bird_now.npy',P); r.snap('birdview','/workspace/now_birdview.png')
" 2>&1 | grep -v "^\[" ; python3 hmap.py bird_now.npy 0.29 0.02 0.12 0.90 | tail -36

# openrua op 123
cat > reach2.py <<'EOF'
import numpy as np
from reach import fk
res={}
for q2 in np.linspace(1.0,1.76,20):
  for q3 in np.linspace(-0.5,0.5,5):
    for q4 in np.linspace(-1.6,-0.07,32):
      for q5 in np.linspace(-0.8,0.8,5):
        for q6 in np.linspace(0.0,3.75,50):
          q=np.array([0,q2,q3,q4,q5,q6,0.785]); p,R=fk(q)
          if 0.975<=p[2]<=1.0 and abs(p[1])<0.05:
            tilt=np.degrees(np.arccos(-R[2,2])); b=int(tilt//10)*10
            if abs(R[1,2])>0.3: continue
            if b not in res or p[0]>res[b][0]: res[b]=(p[0],p[2],tilt,q.round(2))
for b in sorted(res): print(b,res[b])
EOF
python3 reach2.py

# openrua op 124
timeout 600 python3 dragA.py grip 2>&1 | grep -v "^\[" | tail -2; timeout 600 python3 dragA.py to 0.25 0.01 1.03 2>&1 | grep -v "^\[" | tail -3

# openrua op 125
timeout 600 python3 dragA.py to 0.25 0.01 0.995 2>&1 | grep -v "^\[" | tail -3; timeout 300 python3 dragA.py snap p1 2>&1 | tail -1

# openrua op 126
python3 -c "
import cv2
im=cv2.imread('p1_sideview.png'); h,w=im.shape[:2]; print(w,h)
c=im[int(h*0.35):int(h*0.75), int(w*0.05):int(w*0.45)]; cv2.imwrite('p1_side_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('p1_agentview.png'); h,w=im.shape[:2]
c=im[int(h*0.45):int(h*0.95), int(w*0.25):int(w*0.75)]; cv2.imwrite('p1_agent_crop.png',cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 127
for p in "0.245 0.01 0.985" "0.24 0.01 0.975"; do timeout 600 python3 dragA.py to $p 2>&1 | grep "^=="; done; timeout 300 python3 dragA.py snap p2 >/dev/null 2>&1; python3 -c "
import cv2
im=cv2.imread('p2_sideview.png'); h,w=im.shape[:2]
c=im[int(h*0.35):int(h*0.75), int(w*0.05):int(w*0.45)]; cv2.imwrite('p2_side_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 128
timeout 300 python3 -c "
import numpy as np
from rob import *
r=Rob('bird'); P=r.depth_world('birdview',*BIRD); np.save('/workspace/bird_now.npy',P)
" 2>&1 | grep -v "^\[" ; python3 hmap.py bird_now.npy 0.29 0.02 0.12 0.90 | sed -n 10,26p

# openrua op 129
timeout 600 python3 dragA.py to 0.25 0.01 1.03 2>&1 | grep "^=="; for p in "0.256 0.01 1.01" "0.256 0.01 0.99"; do timeout 600 python3 dragA.py to $p 2>&1 | grep -E "^==|FAIL"; done

# openrua op 130
for p in "0.255 0.01 1.0" "0.254 0.01 0.99" "0.252 0.01 0.98"; do timeout 600 python3 dragA.py to $p 2>&1 | grep -E "^==|FAIL"; done; timeout 300 python3 -c "
import numpy as np
from rob import *
r=Rob('bird'); P=r.depth_world('birdview',*BIRD); np.save('/workspace/bird_now.npy',P)
" 2>&1 | grep -v "^\[" ; python3 hmap.py bird_now.npy 0.29 0.02 0.12 0.90 | sed -n 10,26p

# openrua op 131
python3 hmap.py bird_now.npy -0.16 0.27 0.11 0.90 | head -34

# openrua op 132
python3 -c "
import cv2
im=cv2.imread('p2_agentview.png'); h,w=im.shape[:2]
c=im[int(h*0.25):int(h*0.6), int(w*0.55):int(w*0.95)]; cv2.imwrite('B_agent_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('p2_sideview.png'); h,w=im.shape[:2]
c=im[int(h*0.3):int(h*0.7), int(w*0.4):int(w*0.85)]; cv2.imwrite('B_side_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 133
mkdir -p "$(dirname /workspace/potB.py)"
cat > /workspace/potB.py <<'OPENRUA_EOF'
"""Pot B: lid-rim pinch from the lid end (pot lying on the table), lift, hang base-down.
  python3 potB.py grasp        -> open, pre, grasp, close, small lift; prints readings
  python3 potB.py hang         -> lift high and rotate hz down (pot hangs base-down)
  python3 potB.py to X Y Z [hyaz]  -> move TCP with hz down, hy at azimuth hyaz deg (default 0 = +x)
  python3 potB.py fingers | open | snap TAG
"""
import sys, numpy as np
from rob import *
r=Rob("potB")
# pot B axis (base->lid) from birdview: base end (-0.09,0.257), lid band (-0.2135,0.297), z 0.933
U=np.array([-0.95,0.31,0.0]); U/=np.linalg.norm(U)
LID=np.array([-0.2135,0.297,0.933])
HZ=-U                                   # approach from the lid end, pointing at the base
V1=np.array([-U[1],U[0],0.0])           # horizontal, perpendicular to the axis (toward +y side)
al=np.radians(-14); HY=np.cos(al)*V1+np.sin(al)*np.array([0,0,1.0])
R_G=R_from_axes(HZ,HY)
def R_down(hyaz):
    return R_from_axes([0,0,-1],[np.cos(np.radians(hyaz)),np.sin(np.radians(hyaz)),0])
SEEDS=[np.array(s) for s in ([0,-0.8,0,-2.5,0,1.7,0.785],[0.5,0.6,0,-2.0,0,2.6,0.8],[0.8,0.3,-0.3,-2.2,0.3,2.5,1.5],
       [0.6,0.9,-0.4,-1.6,0.5,2.4,0.0],[0.7,0.5,0.2,-2.3,-0.4,2.8,2.0],[0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])]
def solve(p,R):
    for s in [r.arm_q()]+SEEDS:
        q=r.ik(p,R,seed=s,at_tcp=True)
        if q is not None: return q
    raise SystemExit(f"IK FAIL {np.round(p,3)}")
def go(p,R,tag="",tol=0.02,speed=0.15):
    q=solve(p,R); dur=max(2.0,np.abs(q-r.arm_q()).max()/speed)
    code,err=r.move_q(q,dur); pp,RR=r.tcp()
    print(f"== {tag}{np.round(p,3)}: code={code} jerr={err:.4f} tcp={pp.round(3)} hz={RR[:,2].round(2)} fingers={np.round(np.abs(r.fingers()),4)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
cmd=sys.argv[1]
if cmd=="grasp":
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    print("hz",HZ.round(3),"hy",HY.round(3),"hx",R_G[:,0].round(3))
    go(LID-0.12*HZ+np.array([0,0,0.08]),R_G,"high ")
    go(LID-0.12*HZ,R_G,"pre ")
    go(LID-0.05*HZ,R_G,"mid ")
    go(LID,R_G,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(LID+np.array([0,0,0.06]),R_G,"lift6 ")
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
elif cmd=="hang":
    p,R=r.tcp()
    go(np.array([p[0],p[1],1.15]),R,"up ")
    # rotate hz from horizontal to down in two steps about hy, keeping TCP fixed
    hz0=R[:,2]; hy0=R[:,1]
    for ang in (45,90):
        a=np.radians(ang); hz=np.cos(a)*hz0+np.sin(a)*np.array([0,0,-1.0]); hz-=hy0*hz.dot(hy0)
        go(np.array([p[0],p[1],1.15]),R_from_axes(hz,hy0),f"rot{ang} ",speed=0.12)
    print("fingers:",np.round(np.abs(r.fingers()),4))
elif cmd=="to":
    X,Y,Z=map(float,sys.argv[2:5]); hyaz=float(sys.argv[5]) if len(sys.argv)>5 else 0.0
    go(np.array([X,Y,Z]),R_down(hyaz),"to ",speed=0.12)
    f=np.abs(r.fingers())
    if min(f)<0.01: print("!! POT B LOST")
elif cmd=="fingers":
    print(np.round(np.abs(r.fingers()),4))
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="snap":
    for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
print("OK",flush=True)
OPENRUA_EOF

# openrua op 134
timeout 600 python3 park.py 2 2>&1 | grep -v "^\[" | tail -2; timeout 900 python3 potB.py grasp 2>&1 | grep -v "^\["

# openrua op 135
sed -i 's/^U=np.array(\[-0.95,0.31,0.0\])/U=np.array([-0.946,0.319,0.0])/; s/^LID=np.array(\[-0.2135,0.297,0.933\])/LID=np.array([-0.088,0.257,0.933])+0.125*U/; s/^al=np.radians(-14)/al=np.radians(0)/' potB.py && grep -n "^U=\|^LID=\|^al=" potB.py && timeout 900 python3 potB.py grasp 2>&1 | grep -v "^\["

# openrua op 136
cat >> potB.py <<'EOF'
if cmd=="grasp2":
    # top-down pinch across the lid rim: hz down, hy horizontal perpendicular to the pot axis
    C=np.array([-0.088,0.257,0.938])+0.13*U
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    Rt=None
    for hy in (V1,-V1):
        R=R_from_axes([0,0,-1],hy)
        try: solve(C,R); Rt=R; break
        except SystemExit: pass
    if Rt is None: raise SystemExit("no IK for top pinch")
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3))
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
EOF
timeout 600 python3 park.py 2 2>&1 | grep -v "^\[" | tail -1; timeout 900 python3 potB.py grasp2 2>&1 | grep -v "^\["

# openrua op 137
timeout 300 python3 potB.py snap g1 2>&1 | tail -1; python3 -c "
import cv2
im=cv2.imread('g1_agentview.png'); h,w=im.shape[:2]
c=im[int(h*0.2):int(h*0.6), int(w*0.5):int(w*0.95)]; cv2.imwrite('g1_agent_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 138
timeout 600 python3 -c "
import numpy as np
from rob import *
r=Rob('x'); r.gripper(0.04); p,R=r.tcp(); q=r.ik([p[0],p[1],1.15],R,at_tcp=True); print(r.move_q(q,3))
P=r.depth_world('birdview',*BIRD); np.save('/workspace/bird_now.npy',P)
" 2>&1 | grep -v "^\[" ; python3 hmap.py bird_now.npy -0.16 0.27 0.11 0.90 | head -34

# openrua op 139
timeout 600 python3 park.py 2 2>&1 | grep -v "^\[" | tail -1; timeout 300 python3 -c "
import numpy as np
from rob import *
r=Rob('bird'); P=r.depth_world('birdview',*BIRD); np.save('/workspace/bird_now.npy',P)
" 2>&1 | grep -v "^\[" ; python3 hmap.py bird_now.npy -0.16 0.22 0.11 0.90 | head -30

# openrua op 140
sed -i 's/^U=np.array(\[-0.946,0.319,0.0\])/U=np.array([-1.0,0.022,0.0])/; s/^LID=np.array(\[-0.088,0.257,0.933\])+0.125\*U/LID=np.array([-0.067,0.2375,0.933])+0.125*U/; s/    C=np.array(\[-0.088,0.257,0.938\])+0.13\*U/    C=np.array([-0.067,0.2375,0.938])+0.13*U/' potB.py && grep -n "^U=\|^LID=\|C=np" potB.py && timeout 900 python3 potB.py grasp2 2>&1 | grep -v "^\["

# openrua op 141
timeout 300 python3 -c "
from rob import *
r=Rob('s'); r.snap('robot0_eye_in_hand','/workspace/g2_eye.png'); r.snap('agentview','/workspace/g2_agent.png')
" 2>&1 | grep -v "^\["; python3 -c "
import cv2
im=cv2.imread('g2_agent.png'); h,w=im.shape[:2]
c=im[int(h*0.2):int(h*0.6), int(w*0.5):int(w*0.95)]; cv2.imwrite('g2_agent_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 142
cat >> potB.py <<'EOF'
if cmd=="grasp3":
    # pinch across the lid rim with the hand tilted 20 deg toward +y so the -y pad passes under the elevated handle
    C=np.array([-0.197,0.24,0.933]); b=np.radians(20)
    Rt=R_from_axes([0,np.sin(b),-np.cos(b)],[0,np.cos(b),np.sin(b)])
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3),"hz",Rt[:,2].round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C+np.array([0,0,0.02]),Rt,"near ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
EOF
timeout 600 python3 -c "
from rob import *
r=Rob('x'); r.gripper(0.04); p,R=r.tcp(); q=r.ik([p[0],p[1],1.15],R,at_tcp=True); print(r.move_q(q,3))" 2>&1 | grep -v "^\[" | tail -1
timeout 900 python3 potB.py grasp3 2>&1 | grep -v "^\["

# openrua op 143
timeout 300 python3 -c "
from rob import *
r=Rob('s'); r.snap('robot0_eye_in_hand','/workspace/g3_eye.png'); r.snap('frontview','/workspace/g3_front.png')
" 2>&1 | grep -v "^\["; python3 -c "
import cv2
im=cv2.imread('g3_front.png'); h,w=im.shape[:2]
c=im[int(h*0.15):int(h*0.55), int(w*0.55):int(w*1.0)]; cv2.imwrite('g3_front_crop.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 144
cat >> potB.py <<'EOF'
if cmd=="grasp4":
    b=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 25); d=float(sys.argv[3]) if len(sys.argv)>3 else 0.012
    hz=np.array([0,np.sin(b),-np.cos(b)]); hy=np.array([0,np.cos(b),np.sin(b)])
    Rt=R_from_axes(hz,hy); C=np.array([-0.197,0.24,0.933])+d*hz
    print("C",C.round(3),"hy",hy.round(3),"hz",hz.round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C+np.array([0,0,0.02]),Rt,"near ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
EOF
timeout 600 python3 -c "
from rob import *
r=Rob('x'); p,R=r.tcp(); q=r.ik([p[0],p[1],1.15],R,at_tcp=True); print(r.move_q(q,3))" 2>&1 | grep -v "^\[" | tail -1
timeout 900 python3 potB.py grasp4 25 0.012 2>&1 | grep -v "^\["

# openrua op 145
cat >> potB.py <<'EOF'
if cmd=="grasp5":
    # approach along the pot axis from the lid end, hand pitched 25 deg down so the wrist clears the table;
    # pads (planes y=const) clamp the lid rim's equator; handle strut (elevated, -y side) stays 2 cm above the pads
    b=np.radians(25); hz=np.array([np.cos(b),0,-np.sin(b)]); C=np.array([-0.195,0.24,0.933])
    Rt=None
    for hy in ([0,1.0,0],[0,-1.0,0]):
        R=R_from_axes(hz,hy)
        try: solve(C,R); Rt=R; break
        except SystemExit: pass
    if Rt is None: raise SystemExit("no IK for lid-end pinch")
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3),"hz",Rt[:,2].round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C-0.10*hz+np.array([0,0,0.10]),Rt,"high ")
    go(C-0.10*hz,Rt,"pre ")
    go(C-0.04*hz,Rt,"mid ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.03]),Rt,"lift3 ",speed=0.08)
    go(C+np.array([0,0,0.10]),Rt,"lift10 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
EOF
timeout 600 python3 -c "
from rob import *
r=Rob('x'); p,R=r.tcp(); q=r.ik([p[0],p[1],1.15],R,at_tcp=True); print(r.move_q(q,3))" 2>&1 | grep -v "^\[" | tail -1
timeout 900 python3 potB.py grasp5 2>&1 | grep -v "^\["
