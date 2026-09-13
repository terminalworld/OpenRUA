#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v parameter; echo ---; ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 4
timeout 120 python3 tools/perception/px2world.py birdview 320 298; timeout 60 python3 tools/perception/px2world.py birdview 420 298; timeout 60 python3 tools/perception/px2world.py birdview 240 260; timeout 60 python3 tools/perception/px2world.py birdview 320 380

# openrua op 5
cat > pc.py <<'EOF'
#!/usr/bin/env python3
"""Dump world-frame points for a camera: python3 pc.py <cam> [step]. Saves <cam>_pc.npy (H,W,3) world xyz."""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("pc")
buf = Buffer(); TransformListener(buf, node)
d = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
frame = f"{cam}_optical_frame"
import time; end=time.time()+20
while time.time()<end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q=t.transform.rotation; R=qR(q.x,q.y,q.z,q.w)
tr=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
v,u = np.mgrid[0:d.height,0:d.width]
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
P = np.stack([X,Y,depth],-1) @ R.T + tr
np.save(f"{cam}_pc.npy", P)
print("cam pos", tr, "saved", P.shape)
rclpy.shutdown()
EOF
timeout 120 python3 pc.py birdview && timeout 120 python3 pc.py agentview

# openrua op 6
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
# yellow mug region: points above table near (0,0)
m=(z>0.905)&(z<1.05)&(np.abs(x)<0.1)&(np.abs(y)<0.1)
print("yellow mug: n",m.sum(),"x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max(),"z",z[m].min(),z[m].max())
print(" centroid", x[m].mean(), y[m].mean())
# gray mug
m=(z>0.905)&(z<1.05)&(np.abs(x)<0.1)&(np.abs(y-0.35)<0.1)
print("gray mug: x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max(),"z",z[m].max())
# microwave: anything above table with y<-0.1
m=(z>0.905)&(y<-0.1)&(x>-0.5)&(x<0.5)
print("microwave region: x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max(),"z",z[m].min(),z[m].max())
# microwave body top (z>1.08)
m=(z>1.08)&(y<-0.1)&(x>-0.5)&(x<0.5)
print("mw top: x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max(),"z",z[m].mean())
# door: points between 0.92 and 1.08 with y<-0.1
m=(z>0.92)&(z<1.08)&(y<-0.1)&(x>-0.5)&(x<0.5)
print("mid-height: x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max())
# histogram of z for microwave region
import collections
m=(y<-0.1)&(x>-0.5)&(x<0.5)&(z>0.9)
h,e=np.histogram(z[m],bins=20)
for hh,ee in zip(h,e): print(f"{ee:.3f} {hh}")
EOF

# openrua op 7
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
# print world coords at a grid of pixels over the microwave region in agentview image
for v in range(120, 340, 20):
    row=[]
    for u in range(20, 240, 20):
        row.append(f"({x[v,u]:+.2f},{y[v,u]:+.2f},{z[v,u]:.2f})")
    print(v, " ".join(row))
EOF

# openrua op 8
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
m=(z>0.92)&(z<1.2)&(y<-0.36)&(x<0.1)&(x>-0.6)
xs,ys,zs=x[m],y[m],z[m]
print("door pts",m.sum())
# fit: bin by y, print x range
for yb in np.arange(-0.62,-0.34,0.02):
    mm=(ys>=yb)&(ys<yb+0.02)
    if mm.sum(): print(f"y {yb:.2f}: x {xs[mm].min():+.3f}..{xs[mm].max():+.3f} z {zs[mm].min():.2f}..{zs[mm].max():.2f} n {mm.sum()}")
# body
m=(z>1.08)&(z<1.2)&(y>-0.4)&(y<-0.1)&(x<0.1)&(x>-0.6)
print("body top: x",x[m].min(),x[m].max(),"y",y[m].min(),y[m].max())
EOF

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py galleryview; timeout 60 python3 tools/perception/cam_snap.py paperview

# openrua op 10
timeout 120 python3 pc.py robot0_robotview && python3 - <<'EOF'
import numpy as np
P=np.load("robot0_robotview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
for v in range(0, 340, 30):
    row=[]
    for u in range(0, 200, 20):
        row.append(f"({x[v,u]:+.2f},{y[v,u]:+.2f},{z[v,u]:.2f})")
    print(v, " ".join(row))
EOF

# openrua op 11
timeout 120 python3 pc.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
for v in range(0, 400, 40):
    row=[]
    for u in range(440, 640, 20):
        row.append(f"({x[v,u]:+.2f},{y[v,u]:+.2f},{z[v,u]:.2f})")
    print(v, " ".join(row))
EOF
ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 12
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
# look for points on/inside -y face: x in body range, z between table and top
m=(z>0.905)&(z<1.09)&(x>-0.30)&(x<0.03)&(y<-0.30)&(y>-0.5)
print("n",m.sum())
if m.sum():
    for zb in np.arange(0.9,1.1,0.02):
        mm=m&(z>=zb)&(z<zb+0.02)
        if mm.sum(): print(f"z {zb:.2f}: y {y[mm].min():+.3f}..{y[mm].max():+.3f} x {x[mm].min():+.3f}..{x[mm].max():+.3f} n {mm.sum()}")
# -x face
m=(z>0.905)&(z<1.09)&(y>-0.37)&(y<-0.17)&(x<-0.25)&(x>-0.5)
print("n -x face",m.sum())
if m.sum():
    for zb in np.arange(0.9,1.1,0.02):
        mm=m&(z>=zb)&(z<zb+0.02)
        if mm.sum(): print(f"z {zb:.2f}: x {x[mm].min():+.3f}..{x[mm].max():+.3f} y {y[mm].min():+.3f}..{y[mm].max():+.3f} n {mm.sum()}")
EOF

# openrua op 13
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
np.set_printoptions(linewidth=250, precision=2)
# print z grid over microwave footprint (x from -0.45..0.08, y from -0.65..-0.12) 
xs=np.arange(-0.44,0.08,0.02); ys=np.arange(-0.64,-0.12,0.02)
print("      "+" ".join(f"{yy:+.2f}" for yy in ys))
for xb in xs:
    row=[]
    for yb in ys:
        m=(x>=xb)&(x<xb+0.02)&(y>=yb)&(y<yb+0.02)
        row.append(f"{z[m].max():5.2f}" if m.sum() else "  .  ")
    print(f"{xb:+.2f} "+" ".join(row))
EOF

# openrua op 14
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview","robot0_robotview"]:
    P=np.load(f"{cam}_pc.npy")
    x,y,z=P[...,0],P[...,1],P[...,2]
    m=(z>0.905)&(np.abs(x)<0.1)&(np.abs(y)<0.12)
    zs=z[m]
    h,e=np.histogram(zs,bins=np.arange(0.9,1.08,0.01))
    print(cam, " ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
    # top rim: points with z>1.0
    mm=m&(z>1.0)
    if mm.sum(): print("  rim x",x[mm].min(),x[mm].max(),"y",y[mm].min(),y[mm].max(), "centroid", x[mm].mean(), y[mm].mean())
    mm=m&(z>0.95)&(z<1.0)
    if mm.sum(): print("  mid x",x[mm].min(),x[mm].max(),"y",y[mm].min(),y[mm].max())
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
P=np.load("robot0_robotview_pc.npy")
for v in range(90,300,10):
    u=320
    print(v, np.round(P[v,u],3), "  u=280:", np.round(P[v,280],3))
EOF

# openrua op 16
timeout 60 python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("urdf")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,"/robot_description",lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0]); print(len(got[0]))
EOF
grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 '<link name="panda_leftfinger"' robot.urdf | head -40; grep -n 'joint name="panda_hand_joint"' -A6 robot.urdf; grep -n 'panda_hand_tcp\|camera\|eye_in_hand' robot.urdf | head

# openrua op 17
python3 - <<'EOF'
import re
u=open("robot.urdf").read()
for name in ["panda_hand","panda_leftfinger","panda_rightfinger","panda_link8"]:
    m=re.search(rf'<link name="{name}">.*?</link>',u,re.S); print(m.group(0)[:600] if m else name+" none"); print()
for j in ["panda_joint8","panda_hand_joint","panda_finger_joint1","panda_hand_tcp_joint"]:
    m=re.search(rf'<joint name="{j}".*?</joint>',u,re.S); print(m.group(0)[:500] if m else j+" none"); print()
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/

# openrua op 18
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helper: one node, persistent clients.

import rb; R = rb.Robot()
R.joints() -> dict ; R.fk() -> (pos_world, quat) ; R.ik(pos_world, quat) -> [7]
R.move(positions, secs) ; R.move_path([[7]...], secs_each) ; R.grip(width)
R.snap(cam, out) ; R.cloud(cam) -> (H,W,3) world xyz
World<->base: base at (-0.66, 0, 0.912) with identity rotation.
"""
import struct
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x,y,z,w


def frame_quat(hand_z, hand_y):
    """Quaternion for a hand frame whose z (approach) and y (finger) axes
    are the given world vectors."""
    z = np.array(hand_z, float); z /= np.linalg.norm(z)
    y = np.array(hand_y, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return R_to_quat(np.stack([x, y, z], 1))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rb_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped,
                                                    "/servo_node/delta_twist_cmds", 10)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t = time.time()
        while "m" not in self._js and time.time() - t < 20:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World position + quaternion of a link (default hand frame)."""
        q = self.arm_q() if q is None else list(q)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y,
                              p.orientation.z, p.orientation.w])

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + R[:, 2] * M["hand"]["tcp_offset_m"], quat

    def ik(self, pos_world, quat, seed=None, tcp=True, attempts=3):
        """Joint solution for hand (or TCP if tcp=True) at a world pose."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - quat_to_R(quat)[:, 2] * M["hand"]["tcp_offset_m"]
        pos = pos - BASE
        seed = self.arm_q() if seed is None else list(seed)
        last = None
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            last = res.error_code.val if res else None
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        raise RuntimeError(f"IK failed code={last} for {pos_world}")

    # ---------------- acting ----------------
    def move_path(self, qs, secs_each=2.0, first_secs=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for i, q in enumerate(qs):
            t += (first_secs if (i == 0 and first_secs) else secs_each)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        q = np.array(self.arm_q()); err = np.abs(q - np.array(qs[-1])).max()
        print(f"move: code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, q, secs=3.0):
        return self.move_path([q], secs)

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"grip({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, lin, n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        for _ in range(n):
            self.twist_pub.publish(msg); self.spin(0.05)

    # ---------------- cameras ----------------
    def _grab(self, topic, T, timeout=60):
        got = {}
        sub = self.node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
        t = time.time()
        while "m" not in got and time.time() - t < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        return got.get("m")

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        m = self._grab(f"/{cam}/color/image_raw", Image)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(m, "bgr8"))
        return out

    def cloud(self, cam):
        d = self._grab(f"/{cam}/depth/image_raw", Image)
        info = self._grab(f"/{cam}/color/camera_info", CameraInfo)
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{cam}_optical_frame"
        t = time.time()
        while time.time() - t < 20 and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        tr = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = tr.transform.rotation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        t3 = np.array([tr.transform.translation.x, tr.transform.translation.y,
                       tr.transform.translation.z])
        v, u = np.mgrid[0:d.height, 0:d.width]
        P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1) @ R.T + t3
        return P
OPENRUA_EOF

# openrua op 19
timeout 600 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q(); print("q",np.round(q,3))
pos,quat=R.fk(); print("hand world",np.round(pos,3),"quat",np.round(quat,3))
print("tcp",np.round(R.tcp()[0],3))
# settle the world: hold current pose for 2s of sim time
R.move(q, 2.0)
print("gap", R.finger_gap())
R.snap("birdview","/workspace/bird2.png"); R.snap("agentview","/workspace/agent2.png")
P=R.cloud("birdview"); np.save("/workspace/bird2_pc.npy",P)
P=R.cloud("agentview"); np.save("/workspace/agent2_pc.npy",P)
EOF

# openrua op 20
ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -4; ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -4; timeout 120 python3 - <<'EOF'
import rclpy
from moveit_msgs.srv import GetPositionFK
rclpy.init(); n=rclpy.create_node("fkt")
c=n.create_client(GetPositionFK,"/compute_fk"); c.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link1"]
req.robot_state.joint_state.name=["panda_joint%d"%i for i in range(1,8)]
req.robot_state.joint_state.position=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
f=c.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code)
for ps in r.pose_stamped: print(ps.header.frame_id, ps.pose.position)
EOF

# openrua op 21
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
rb.BASE = np.zeros(3)  # test: FK already in world
R=rb.Robot()
q=R.arm_q()
pos,quat=R.fk(); print("hand world (FK)",np.round(pos,3))
try:
    s=R.ik(pos, quat, tcp=False); print("IK world-coords ->",np.round(s,3), "diff", np.round(np.abs(np.array(s)-q).max(),3))
except Exception as e: print("IK world failed",e)
try:
    s=R.ik(pos-np.array([-0.66,0,0.912]), quat, tcp=False); print("IK base-coords ->",np.round(s,3), "diff", np.round(np.abs(np.array(s)-q).max(),3))
except Exception as e: print("IK base failed",e)
EOF

# openrua op 22
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.zeros(3)  # FK\/IK services already work in the world frame (verified)/' rb.py && grep -n "^BASE" rb.py && timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q()
pos,quat=R.fk()
s=R.ik(pos, quat, tcp=False)
p2,q2=R.fk(s); print("sol fk",np.round(p2,4),np.round(q2,3)," orig",np.round(pos,4),np.round(quat,3))
# check finger positions
print(R.joints())
EOF

# openrua op 23
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
from scipy.spatial.transform import Rotation as Rot
R=rb.Robot()
q=R.arm_q()
ph,qh=R.fk(q,"panda_hand"); p8,q8=R.fk(q,"panda_link8")
print("hand",np.round(qh,3),"link8",np.round(q8,3))
# predicted link8 = hand * Rz(45deg)
pred=(Rot.from_quat(qh)*Rot.from_rotvec([0,0,np.pi/4])).as_quat()
print("pred link8",np.round(pred,3))
EOF

# openrua op 24
python3 - <<'EOF'
import re
s=open("/workspace/rb.py").read()
s=s.replace('''        pos = pos - BASE
        seed = self.arm_q() if seed is None else list(seed)''','''        pos = pos - BASE
        # IK tip link is panda_link8 = hand rotated +45deg about hand z
        from scipy.spatial.transform import Rotation as Rot
        quat = (Rot.from_quat(quat) * Rot.from_rotvec([0, 0, np.pi / 4])).as_quat()
        seed = self.arm_q() if seed is None else list(seed)''')
open("/workspace/rb.py","w").write(s)
EOF
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q(); pos,quat=R.fk()
s=R.ik(pos,quat,tcp=False); p2,q2=R.fk(s)
print("orig",np.round(quat,3),"sol",np.round(q2,3),"pos err",np.round(np.abs(p2-pos).max(),4))
EOF

# openrua op 25
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/bird2_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
m=(z>0.905)&(np.abs(x)<0.1)&(np.abs(y)<0.12)
h,e=np.histogram(z[m],bins=np.arange(0.9,1.1,0.01)); print("mug z hist", " ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
mm=m&(z>0.98); print("rim centroid",x[mm].mean(),y[mm].mean(),"x",x[mm].min(),x[mm].max(),"y",y[mm].min(),y[mm].max())
# door
m=(z>0.92)&(z<1.2)&(y<-0.36)&(x<0.1)&(x>-0.6)
for yb in np.arange(-0.64,-0.34,0.02):
    mm=m&(y>=yb)&(y<yb+0.02)
    if mm.sum(): print(f"y {yb:.2f}: x {x[mm].min():+.3f}..{x[mm].max():+.3f} z {z[mm].min():.2f}..{z[mm].max():.2f} n {mm.sum()}")
P=np.load("/workspace/agent2_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
m=(z>0.905)&(np.abs(x)<0.1)&(np.abs(y)<0.12)
h,e=np.histogram(z[m],bins=np.arange(0.9,1.1,0.01)); print("agent mug z hist", " ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
EOF

# openrua op 26
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qd=rb.frame_quat([0,0,-1],[1,0,0])   # hand pointing down, fingers along world x
print("quat down",np.round(qd,3))
target=np.array([-0.003,0.008,1.10])
s=R.ik(target,qd)
print("ik",np.round(s,3))
R.move(s,4.0)
p,qq=R.tcp(); print("tcp now",np.round(p,4),np.round(qq,3))
R.grip(0.04)
R.snap("robot0_eye_in_hand","/workspace/eih1.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih1_pc.npy",P)
EOF

# openrua op 27
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
s=[0.055,0.478,-0.048,-1.773,0.028,2.25,-0.791]
q=R.arm_q(); print("now",np.round(q,3)); print("diff",np.round(np.array(q)-s,3))
R.move(s,3.0)
q=R.arm_q(); print("now",np.round(q,3))
p,qq=R.tcp(); print("tcp now",np.round(p,4),np.round(qq,3))
EOF

# openrua op 28
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
s=[0.055,0.478,-0.048,-1.773,0.028,2.25,-0.791]
R.move(s,4.0)
q=R.arm_q(); print("now",np.round(q,3), "j7 diff", round(q[6]-s[6],4))
p,qq=R.tcp(); print("tcp now",np.round(p,4),np.round(qq,3))
R.snap("robot0_eye_in_hand","/workspace/eih1.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih1_pc.npy",P)
EOF

# openrua op 29
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/eih1_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(z)
m=ok&(z>0.985)&(z<1.02)&(np.abs(x)<0.12)&(np.abs(y)<0.12)
print("rim pts",m.sum())
xs,ys=x[m],y[m]
# fit circle (least squares)
A=np.c_[2*xs,2*ys,np.ones_like(xs)]; b=xs**2+ys**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print("circle fit center",c[0],c[1],"r",r)
# exclude handle region (y < center - r*0.9) and refit
keep=ys>c[1]-0.9*r
xs2,ys2=xs[keep],ys[keep]
A=np.c_[2*xs2,2*ys2,np.ones_like(xs2)]; b=xs2**2+ys2**2
c2=np.linalg.lstsq(A,b,rcond=None)[0]; r2=np.sqrt(c2[2]+c2[0]**2+c2[1]**2)
print("refit center",c2[0],c2[1],"r",r2)
# mug body profile: radius vs height using all mug points
m=ok&(z>0.905)&(z<1.02)&(np.hypot(x-c2[0],y-c2[1])<0.09)
for zb in np.arange(0.90,1.02,0.01):
    mm=m&(z>=zb)&(z<zb+0.01)
    if mm.sum():
        rr=np.hypot(x[mm]-c2[0],y[mm]-c2[1])
        # body radius: exclude handle direction (-y)
        body=mm&(y>c2[1]-0.02)
        rb_=np.hypot(x[body]-c2[0],y[body]-c2[1]) if body.sum() else np.array([np.nan])
        print(f"z {zb:.2f}: n {mm.sum()} rmax_all {rr.max():.3f} body r max {np.nanmax(rb_):.3f}")
EOF

# openrua op 30
python3 - <<'EOF'
import numpy as np
cx,cy=-0.0037,0.0147
for f in ["agent2_pc.npy"]:
    P=np.load("/workspace/"+f); x,y,z=P[...,0],P[...,1],P[...,2]
    m=(z>0.905)&(z<1.01)&(np.hypot(x-cx,y-cy)<0.1)
    print(f)
    for zb in np.arange(0.90,1.01,0.01):
        mm=m&(z>=zb)&(z<zb+0.01)
        if mm.sum()<5: continue
        # outer wall facing camera (+x side): points with x>cx+0.02 ; take radius stats
        side=mm&(x>cx+0.01)&(np.abs(y-cy)<0.015)
        r=(x[side]-cx) if side.sum() else np.array([np.nan])
        # also y extent to get center estimate: max y and min y (excluding handle -y)
        print(f"z {zb:.2f}: n {mm.sum()} +x wall x-cx: {np.nanmedian(r):.4f}  y range {y[mm].min()-cy:+.3f}..{y[mm].max()-cy:+.3f}")
EOF

# openrua op 31
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
# first lift straight up to a safe height, then go look at the opening from -y
qd=rb.frame_quat([0,0,-1],[1,0,0])
s1=R.ik([-0.003,0.008,1.25],qd); R.move(s1,3.0)
qh=rb.frame_quat([0,1,0],[1,0,0])   # hand pointing +y, fingers along x, camera above hand
s2=R.ik([-0.17,-0.60,1.05],qh,seed=s1)
print("ik2",np.round(s2,3))
R.move_path([s2],4.0)
c,e=R.move(s2,3.0) if False else (0,0)
p,q=R.tcp(); print("tcp",np.round(p,3),np.round(q,3))
R.snap("robot0_eye_in_hand","/workspace/eih2.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih2_pc.npy",P)
EOF

# openrua op 32
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q0=R.arm_q()
for tgt in ([-0.20,-0.52,1.05],[-0.20,-0.50,1.08],[-0.22,-0.48,1.06]):
    for hz in ([0,1,0],[0,0.94,-0.34]):
        qh=rb.frame_quat(hz,[1,0,0])
        try:
            s=R.ik(tgt,qh,seed=q0,attempts=2); print("OK",tgt,hz,np.round(s,3)); break
        except Exception as e: print("fail",tgt,hz)
    else: continue
    break
R.move(s,4.0)
p,q=R.tcp(); print("tcp",np.round(p,3),np.round(q,3))
R.snap("robot0_eye_in_hand","/workspace/eih2.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih2_pc.npy",P)
EOF

# openrua op 33
python3 - <<'EOF'
s=open("/workspace/rb.py").read()
s=s.replace('''        seed = self.arm_q() if seed is None else list(seed)
        last = None
        for _ in range(attempts):''','''        seed0 = self.arm_q() if seed is None else list(seed)
        rng = np.random.default_rng(0)
        lim = np.array(FJT["limits_rad"])
        last = None
        for k in range(attempts):
            seed = seed0 if k == 0 else list(rng.uniform(lim[:, 0], lim[:, 1]))''')
open("/workspace/rb.py","w").write(s)
EOF
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q0=R.arm_q()
sol=None
for tgt in ([-0.20,-0.52,1.05],[-0.20,-0.48,1.08]):
    for hz,hy in (([0,1,0],[1,0,0]),([0,1,0],[0,0,1]),([0,0.94,-0.34],[1,0,0]),([0,0.94,-0.34],[0,0,1])):
        qh=rb.frame_quat(hz,hy)
        try:
            s=R.ik(tgt,qh,seed=q0,attempts=8); print("OK",tgt,hz,hy,np.round(s,3)); sol=s; break
        except Exception as e: print("fail",tgt,hz,hy)
    if sol: break
if sol:
    R.move(sol,4.0)
    p,q=R.tcp(); print("tcp",np.round(p,3),np.round(q,3))
    R.snap("robot0_eye_in_hand","/workspace/eih2.png")
    P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih2_pc.npy",P)
EOF

# openrua op 34
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q0=R.arm_q()
tests=[("down @ front of opening",[-0.17,-0.50,1.10],rb.frame_quat([0,0,-1],[1,0,0])),
       ("down @ front of opening low",[-0.17,-0.50,0.98],rb.frame_quat([0,0,-1],[1,0,0])),
       ("+y horiz close",[-0.30,-0.30,1.05],rb.frame_quat([0,1,0],[1,0,0])),
       ("+y horiz near base",[-0.40,-0.20,1.10],rb.frame_quat([0,1,0],[1,0,0])),
       ("+x horiz",[-0.10,-0.40,1.05],rb.frame_quat([1,0,0],[0,1,0])),
       ("tilt40 @ front",[-0.17,-0.45,1.0],rb.frame_quat([0,np.sin(0.7),-np.cos(0.7)],[1,0,0])),
       ("tilt40 @ mug",[0.0,-0.07,0.95],rb.frame_quat([0,np.sin(0.7),-np.cos(0.7)],[1,0,0])),
      ]
for name,tgt,qh in tests:
    try:
        s=R.ik(tgt,qh,seed=q0,attempts=6); print("OK  ",name,np.round(s,2))
    except Exception as e: print("FAIL",name)
EOF

# openrua op 35
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q0=R.arm_q()
qh=rb.frame_quat([0,np.sin(0.6),-np.cos(0.6)],[1,0,0])
s=R.ik([-0.17,-0.50,1.08],qh,seed=q0,attempts=6); print(np.round(s,3))
R.move(s,4.0)
p,q=R.tcp(); print("tcp",np.round(p,3),np.round(q,3))
R.snap("robot0_eye_in_hand","/workspace/eih2.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih2_pc.npy",P)
R.snap("agentview","/workspace/agent3.png")
EOF

# openrua op 36
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
s=[-0.856,1.103,-0.127,-0.736,0.464,1.341,-1.736]
q=R.arm_q(); print("diff",np.round(np.array(q)-s,3))
R.move(s,4.0)
q=R.arm_q(); print("diff",np.round(np.array(q)-s,3))
p,qq=R.tcp(); print("tcp",np.round(p,3),np.round(qq,3))
R.snap("robot0_eye_in_hand","/workspace/eih2.png")
P=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih2_pc.npy",P)
R.snap("agentview","/workspace/agent3.png")
EOF

# openrua op 37
python3 - <<'EOF'
s=open("/workspace/rb.py").read()
s=s.replace('''    def move(self, q, secs=3.0):
        return self.move_path([q], secs)''','''    def move(self, q, secs=3.0, tol=0.01, retries=2):
        code, err = self.move_path([q], secs)
        while err > tol and retries > 0:   # controller lag: resend converges
            retries -= 1
            code, err = self.move_path([q], max(2.0, secs / 2))
        return code, err''')
open("/workspace/rb.py","w").write(s)
EOF
echo ok

# openrua op 38
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/eih2_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
np.set_printoptions(linewidth=200)
# sample grid over image
for v in range(60, 300, 20):
    print(v, " ".join(f"({x[v,u]:+.2f},{y[v,u]:+.2f},{z[v,u]:.3f})" for u in range(140, 500, 40)))
EOF

# openrua op 39
python3 - <<'EOF'
import numpy as np
P=np.load("/workspace/eih1_pc.npy")
x,y,z=P[...,0],P[...,1],P[...,2]
cx,cy=-0.0037,0.0147
ok=np.isfinite(z)
h=ok&(z>0.905)&(z<0.995)&(y<cy-0.045)&(y>cy-0.12)&(np.abs(x-cx)<0.03)
print("handle pts",h.sum())
print("x range",x[h].min(),x[h].max()," mean",x[h].mean())
print("y range",y[h].min(),y[h].max())
print("z range",z[h].min(),z[h].max())
for zb in np.arange(0.90,1.0,0.01):
    mm=h&(z>=zb)&(z<zb+0.01)
    if mm.sum(): print(f"z {zb:.2f}: n {mm.sum()} y {y[mm].min():+.4f}..{y[mm].max():+.4f}  x {x[mm].min():+.4f}..{x[mm].max():+.4f}")
# what does the top view see near the handle: the top arm of the handle
h2=ok&(z>0.95)&(z<1.0)&(y<cy-0.045)&(y>cy-0.1)&(np.abs(x-cx)<0.03)
print("upper handle x mean", x[h2].mean() if h2.sum() else None)
EOF

# openrua op 40
python3 - <<'EOF'
import numpy as np
for f in ["agent2_pc.npy","bird2_pc.npy"]:
    P=np.load("/workspace/"+f); x,y,z=P[...,0],P[...,1],P[...,2]
    cx,cy=-0.0037,0.0147
    h=(z>0.905)&(z<0.999)&(y<-0.045)&(y>-0.12)&(np.abs(x-cx)<0.04)
    print(f,"handle pts",h.sum())
    for zb in np.arange(0.90,1.0,0.01):
        mm=h&(z>=zb)&(z<zb+0.01)
        if mm.sum(): print(f"  z {zb:.2f}: n {mm.sum():4d} y {y[mm].min():+.4f}..{y[mm].max():+.4f}  x {x[mm].min():+.4f}..{x[mm].max():+.4f}")
EOF

# openrua op 41
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(50)
hz=np.array([0,np.sin(th),-np.cos(th)]); hy=np.array([1,0,0])
qh=rb.frame_quat(hz,hy)
target=np.array([-0.004,-0.070,0.955])
pre=target-0.08*hz
print("pre",np.round(pre,3))
R.grip(0.04)
s_pre=R.ik(pre,qh,attempts=6); print("s_pre",np.round(s_pre,3))
R.move(s_pre,5.0)
p,q=R.tcp(); print("tcp pre",np.round(p,4),np.round(q,3))
# descend along hz in 4 waypoints
qs=[]; seed=s_pre
for f in (0.25,0.5,0.75,1.0):
    wp=pre+f*(target-pre); s=R.ik(wp,qh,seed=seed,attempts=1); qs.append(s); seed=s
    print("wp",np.round(wp,3),"dq",np.round(np.abs(np.array(s)-np.array(seed)).max(),3))
R.move_path(qs,0.8)
p,q=R.tcp(); print("tcp at grasp",np.round(p,4),np.round(q,3))
R.snap("robot0_eye_in_hand","/workspace/eih3.png"); R.snap("agentview","/workspace/agent4.png")
EOF

# openrua op 42
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=np.array(R.arm_q()); print("q now",np.round(q,3))
s_pre=np.array([-0.449,0.904,0.065,-1.378,0.754,1.783,-1.545])
print("err per joint vs s_pre",np.round(q-s_pre,3))
print("gap",R.finger_gap())
p,qq=R.fk(); print("hand",np.round(p,3)); print("tcp",np.round(R.tcp()[0],3))
EOF

# openrua op 43
timeout 600 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q0=R.arm_q()
p,quat=R.fk()
# retreat: same orientation, hand straight up and out (-y)
for tgt in ([-0.106,-0.45,1.15],[-0.106,-0.45,1.32]):
    s=R.ik(np.array(tgt)+np.array([0,0,0]),quat,seed=q0,tcp=False,attempts=3); R.move(s,3.0); q0=s
    print("hand",np.round(R.fk()[0],3))
P=R.cloud("birdview"); np.save("/workspace/bird3_pc.npy",P)
P0=np.load("/workspace/bird2_pc.npy")
def mw(P):
    z=P[...,2]; m=(z>1.05)&(z<1.15)&(P[...,0]>-0.35)&(P[...,0]<0.1)&(P[...,1]<-0.1)&(P[...,1]>-0.7)
    pts=P[m]; return pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),len(pts)
print("mw before",np.round(mw(P0),3)); print("mw after ",np.round(mw(P),3))
def mug(P):
    m=(P[...,2]>0.95)&(P[...,2]<1.01)&(np.abs(P[...,0])<0.15)&(np.abs(P[...,1])<0.15)
    pts=P[m]; return pts[:,0].mean(),pts[:,1].mean(),len(pts)
print("mug before",np.round(mug(P0),3)); print("mug after ",np.round(mug(P),3))
R.snap("birdview","/workspace/bird3.png")
EOF

# openrua op 44
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(50)
hz=np.array([0,np.sin(th),-np.cos(th)]); hy=np.array([1,0,0])
qh=rb.frame_quat(hz,hy)
target=np.array([-0.004,-0.070,0.955])
pre=target-0.08*hz
high=pre+np.array([0,0,0.25])
q0=R.arm_q()
s_high=R.ik(high,qh,seed=q0,attempts=6); print("s_high",np.round(s_high,3))
R.move(s_high,5.0); print("tcp",np.round(R.tcp()[0],3))
s_pre=R.ik(pre,qh,seed=s_high,attempts=1); print("s_pre",np.round(s_pre,3), "dq",np.round(np.array(s_pre)-np.array(s_high),2))
R.move(s_pre,4.0); print("tcp pre",np.round(R.tcp()[0],4))
qs=[]; seed=s_pre
for f in (0.25,0.5,0.75,1.0):
    wp=pre+f*(target-pre); s=R.ik(wp,qh,seed=seed,attempts=1); qs.append(s); seed=s
R.move_path(qs,0.8)
q=np.array(R.arm_q()); print("err",np.round(np.abs(q-np.array(qs[-1])).max(),4))
p,qq=R.tcp(); print("tcp at grasp",np.round(p,4))
R.snap("robot0_eye_in_hand","/workspace/eih3.png"); R.snap("agentview","/workspace/agent4.png")
EOF

# openrua op 45
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=np.array(R.arm_q()); print("q",np.round(q,3))
s_pre=np.array([-0.251,0.92,-0.216,-1.36,0.952,1.681,-1.617]); print("err vs s_pre",np.round(q-s_pre,3))
for l in ["panda_link5","panda_link6","panda_link7","panda_link8","panda_hand","panda_leftfinger","panda_rightfinger"]:
    print(l,np.round(R.fk(link=l)[0],3))
EOF

# openrua op 46
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
cx,cy=-0.0037,0.0147
gx,gy=cx+0.0494,cy
q0=R.arm_q()
# go up first, then over the rim
s1=R.ik([gx,gy,1.12],qv,seed=q0,attempts=6); R.move(s1,4.0); print("tcp",np.round(R.tcp()[0],4))
R.grip(0.04)
s2=R.ik([gx,gy,1.03],qv,seed=s1,attempts=1); s3=R.ik([gx,gy,0.975],qv,seed=s2,attempts=1)
R.move_path([s2,s3],1.5)
q=np.array(R.arm_q()); print("err",np.round(np.abs(q-np.array(s3)).max(),4),"tcp",np.round(R.tcp()[0],4))
if np.abs(q-np.array(s3)).max()>0.01: R.move(s3,2.0); print("tcp",np.round(R.tcp()[0],4))
gap=R.grip(0.0)
R.snap("agentview","/workspace/agent5.png"); R.snap("robot0_eye_in_hand","/workspace/eih4.png")
EOF

# openrua op 47
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
q0=R.arm_q(); p0=R.tcp()[0]
s1=R.ik([p0[0],p0[1],1.05],qv,seed=q0,attempts=1); R.move(s1,2.0); print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
R.snap("agentview","/workspace/agent6.png")
nx,ny=0.0457,0.15
s2=R.ik([nx,ny,1.05],qv,seed=s1,attempts=1); R.move(s2,3.0); print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
s3=R.ik([nx,ny,0.978],qv,seed=s2,attempts=1); R.move(s3,2.0); print("tcp",np.round(R.tcp()[0],4))
R.grip(0.04)
s4=R.ik([nx,ny,1.08],qv,seed=s3,attempts=1); R.move(s4,2.0); print("tcp",np.round(R.tcp()[0],4))
P=R.cloud("birdview"); np.save("/workspace/bird4_pc.npy",P)
m=(P[...,2]>0.95)&(P[...,2]<1.01)&(np.abs(P[...,0])<0.2)&(P[...,1]>0.0)&(P[...,1]<0.3)
pts=P[m]; print("mug rim pts",len(pts),"center",np.round(pts[:,:2].mean(0),4),"zmax",round(pts[:,2].max(),3))
R.snap("agentview","/workspace/agent7.png")
EOF

# openrua op 48
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
s=R.ik([-0.15,-0.05,1.32],qv,attempts=4); R.move(s,3.0); print("tcp",np.round(R.tcp()[0],3))
P=R.cloud("birdview"); np.save("/workspace/bird5_pc.npy",P)
A=R.cloud("agentview"); np.save("/workspace/agent8_pc.npy",A)
R.snap("birdview","/workspace/bird5.png"); R.snap("agentview","/workspace/agent8.png")
def rim(P):
    m=(P[...,2]>0.985)&(P[...,2]<1.02)&(np.abs(P[...,0])<0.2)&(P[...,1]>0.0)&(P[...,1]<0.3)
    pts=P[m][:,:2]
    # circle fit
    x,y=pts[:,0],pts[:,1]; Amat=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(Amat,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    return len(pts),np.round(c[:2],4),round(r,4)
print("bird rim",rim(P)); print("agent rim",rim(A))
# handle: points with z 0.92-0.99 outside body radius, near mug
for name,Q in (("bird",P),("agent",A)):
    m=(Q[...,2]>0.915)&(Q[...,2]<0.995)&(np.abs(Q[...,0])<0.1)&(Q[...,1]>0.0)&(Q[...,1]<0.3)
    pts=Q[m]; c=rim(Q)[1]; d=np.linalg.norm(pts[:,:2]-c,axis=1); h=pts[d>0.056]
    if len(h): print(name,"handle pts",len(h),"x",np.round([h[:,0].min(),h[:,0].max()],4),"y",np.round([h[:,1].min(),h[:,1].max()],4),"z",np.round([h[:,2].min(),h[:,2].max()],3))
    if len(h):
        bar=h[(h[:,2]>0.935)&(h[:,2]<0.975)]
        if len(bar): print(name,"bar y",np.round([bar[:,1].min(),bar[:,1].max()],4),"x",np.round([bar[:,0].min(),bar[:,0].max()],4))
EOF

# openrua op 49
timeout 300 python3 -u - <<'EOF'
import numpy as np
P=np.load("/workspace/bird5_pc.npy"); A=np.load("/workspace/agent8_pc.npy")
def rim(Q,y0=0.05,y1=0.25):
    m=(Q[...,2]>0.985)&(Q[...,2]<1.02)&(np.abs(Q[...,0])<0.15)&(Q[...,1]>y0)&(Q[...,1]<y1)
    pts=Q[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(Am,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); return len(pts),np.round(c[:2],4),round(r,4)
for n,Q in (("bird",P),("agent",A)):
    k,c,r=rim(Q); print(n,"rim",k,c,r)
    m=(Q[...,2]>0.915)&(Q[...,2]<0.995)&(np.abs(Q[...,0]-c[0])<0.08)&(Q[...,1]>c[1]-0.12)&(Q[...,1]<c[1]+0.08)
    pts=Q[m]; d=np.linalg.norm(pts[:,:2]-c,axis=1); h=pts[d>0.054]
    print(n,"handle",len(h),"x",np.round([h[:,0].min(),h[:,0].max()],4),"y",np.round([h[:,1].min(),h[:,1].max()],4),"z",np.round([h[:,2].min(),h[:,2].max()],3))
    for z0,z1 in ((0.935,0.975),(0.975,0.995),(0.915,0.935)):
        b=h[(h[:,2]>z0)&(h[:,2]<z1)]
        if len(b): print(f"   z{z0}-{z1}: n={len(b)} x[{b[:,0].min():.4f},{b[:,0].max():.4f}] y[{b[:,1].min():.4f},{b[:,1].max():.4f}]")
    # body -y extent at z 0.95
    bd=pts[(d<0.054)&(pts[:,2]>0.945)&(pts[:,2]<0.955)]
    print(n,"body@z0.95 y-min",round(bd[:,1].min(),4),"x range",np.round([bd[:,0].min(),bd[:,0].max()],4))
EOF

# openrua op 50
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
s=R.ik([0.005,0.10,1.15],qv,attempts=4); R.move(s,3.0); print("tcp",np.round(R.tcp()[0],3))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih5_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih5.png")
Q=E
m=(Q[...,2]>0.99)&(Q[...,2]<1.02)&(np.abs(Q[...,0])<0.15)&(Q[...,1]>0.05)&(Q[...,1]<0.25)
pts=Q[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
c=np.linalg.lstsq(Am,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); print("eih rim",len(pts),np.round(c[:2],4),round(r,4))
m=(Q[...,2]>0.915)&(Q[...,2]<0.995)&(np.abs(Q[...,0]-c[0])<0.08)&(Q[...,1]>c[1]-0.12)&(Q[...,1]<c[1]+0.08)
pts=Q[m]; d=np.linalg.norm(pts[:,:2]-c[:2],axis=1); h=pts[d>0.054]
print("handle",len(h),"x",np.round([h[:,0].min(),h[:,0].max()],4),"y",np.round([h[:,1].min(),h[:,1].max()],4),"z",np.round([h[:,2].min(),h[:,2].max()],3))
for z0,z1 in ((0.935,0.975),(0.975,0.995),(0.915,0.935)):
    bb=h[(h[:,2]>z0)&(h[:,2]<z1)]
    if len(bb): print(f"   z{z0}-{z1}: n={len(bb)} x[{bb[:,0].min():.4f},{bb[:,0].max():.4f}] y[{bb[:,1].min():.4f},{bb[:,1].max():.4f}]")
# top of handle arm: z max region
top=h[h[:,2]>0.98]; print("top arm x mean",round(top[:,0].mean(),4),"y range",np.round([top[:,1].min(),top[:,1].max()],4))
EOF

# openrua op 51
timeout 60 ros2 service list 2>/dev/null | grep -i -E "planning_scene|scene" ; timeout 30 ros2 topic list | grep -i -E "collision|scene"

# openrua op 52
cat >> /workspace/rb.py <<'EOF'


# ---------------- planning scene helpers ----------------
def _box(cid, center, size, yaw=0.0, frame="world"):
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    co = CollisionObject(); co.header.frame_id = frame; co.id = cid
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, center)
    pose.orientation.z = float(np.sin(yaw / 2)); pose.orientation.w = float(np.cos(yaw / 2))
    co.primitives.append(prim); co.primitive_poses.append(pose); co.operation = CollisionObject.ADD
    return co


def apply_scene(R, objs):
    from moveit_msgs.srv import ApplyPlanningScene
    cli = R.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.world.collision_objects = objs
    fut = cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    return fut.result().success


def ik_free(R, pos, quat, seed=None, tcp=True, attempts=3):
    """IK that must be collision-free against the planning scene; returns None if impossible."""
    from scipy.spatial.transform import Rotation as Rot
    p = np.array(pos, float)
    if tcp:
        p = p - quat_to_R(quat)[:, 2] * M["hand"]["tcp_offset_m"]
    q8 = (Rot.from_quat(quat) * Rot.from_rotvec([0, 0, np.pi / 4])).as_quat()
    seed0 = R.arm_q() if seed is None else list(seed)
    rng = np.random.default_rng(1); lim = np.array(FJT["limits_rad"]); last = None
    for k in range(attempts):
        sd = seed0 if k == 0 else list(rng.uniform(lim[:, 0], lim[:, 1]))
        req = GetPositionIK.Request(); req.ik_request.group_name = M["planning"]["group"]
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q8)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in sd]
        req.ik_request.avoid_collisions = True; req.ik_request.timeout.sec = 2
        fut = R.ik_cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=60)
        res = fut.result(); last = res.error_code.val if res else None
        if res is not None and res.error_code.val == 1:
            sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            return [sol[j] for j in ARM]
    print("ik_free failed", last, np.round(pos, 3)); return None


def state_valid(R, q):
    """Collision check of an arm configuration via /check_state_validity if present."""
    from moveit_msgs.srv import GetStateValidity
    if not hasattr(R, "_sv"):
        R._sv = R.node.create_client(GetStateValidity, "/check_state_validity")
        if not R._sv.wait_for_service(5):
            return None
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    req.robot_state.joint_state.name = list(ARM); req.robot_state.joint_state.position = [float(v) for v in q]
    fut = R._sv.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    r = fut.result()
    return r.valid, [(c.contact_body_1, c.contact_body_2) for c in r.contacts]
EOF
timeout 600 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
yaw=np.arctan2(-0.24,-0.07)
objs=[rb._box("mw_left",(-0.28,-0.26,1.0035),(0.04,0.20,0.207)),
      rb._box("mw_right",(-0.005,-0.26,1.0035),(0.11,0.20,0.207)),
      rb._box("mw_back",(-0.125,-0.17,1.0035),(0.35,0.02,0.207)),
      rb._box("mw_top",(-0.125,-0.26,1.0985),(0.35,0.20,0.017)),
      rb._box("mw_floor",(-0.125,-0.26,0.922),(0.35,0.20,0.044)),
      rb._box("door",(-0.345,-0.49,1.005),(0.25,0.03,0.21),yaw=yaw),
      rb._box("table",(0.15,0.0,0.85),(1.1,1.4,0.1)),
      rb._box("gray_mug",(0.0,0.35,0.95),(0.10,0.10,0.10))]
print("apply",rb.apply_scene(R,objs))
print("valid now",rb.state_valid(R,R.arm_q()))
qv=rb.frame_quat([0,0,-1],[1,0,0])
# sanity: pose inside microwave body should fail; pose in free space should succeed
print("free-space:",rb.ik_free(R,[0.0,0.1,1.10],qv) is not None)
print("inside mw body:",rb.ik_free(R,[-0.28,-0.26,1.0],qv) is not None)
EOF

# openrua op 53
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
print("apply",rb.apply_scene(R,[rb._box("ymug",(0.0095,0.1495,0.95),(0.085,0.085,0.10))]))
bar=np.array([0.007,0.067,0.955])
q0=R.arm_q()
for deg in (50,45,40,35):
    th=np.deg2rad(deg); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
    s=rb.ik_free(R,bar,qh,seed=q0,attempts=4)
    print(deg,"grasp ok" if s is not None else "grasp BLOCKED", np.round(s,3) if s is not None else "")
    if s is not None:
        sp=rb.ik_free(R,bar-0.08*hz,qh,seed=s,attempts=2); print("   pre ok" if sp is not None else "   pre BLOCKED")
    # insertion pose check: mug center (-0.17,y_m), TCP = center + (0,-0.083,0.105)
    for y_m in (-0.235,-0.25,-0.265,-0.28):
        tcp=np.array([-0.17,y_m-0.083,1.005])
        si=rb.ik_free(R,tcp,qh,seed=q0,attempts=3)
        print(f"   insert y_m={y_m}:", "ok" if si is not None else "BLOCKED")
EOF

# openrua op 54
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(45); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
bar=np.array([0.007,0.067,0.955]); pre=bar-0.08*hz
q0=R.arm_q()
R.grip(0.04)
s_pre=rb.ik_free(R,pre,qh,seed=q0,attempts=4)
# a high intermediate above pre to avoid sweeping through the mug
s_hi=rb.ik_free(R,pre+[0,0,0.12],qh,seed=s_pre,attempts=3)
R.move(s_hi,4.0); print("tcp",np.round(R.tcp()[0],4))
R.move(s_pre,3.0); print("tcp pre",np.round(R.tcp()[0],4))
qs=[]; seed=s_pre
for f in (0.25,0.5,0.75,1.0):
    s=R.ik(pre+f*(bar-pre),qh,seed=seed,attempts=1); qs.append(s); seed=s
R.move_path(qs,0.8)
q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
if err>0.01: R.move(qs[-1],2.0)
p,_=R.tcp(); print("tcp grasp",np.round(p,4))
R.snap("robot0_eye_in_hand","/workspace/eih6.png")
gap=R.grip(0.0)
R.snap("agentview","/workspace/agent9.png")
EOF

# openrua op 55
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
from moveit_msgs.msg import CollisionObject
R=rb.Robot()
co=rb._box("ymug",(0,0,0),(0.01,0.01,0.01)); co.operation=CollisionObject.REMOVE
print("remove ymug",rb.apply_scene(R,[co]))
th=np.deg2rad(45); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
q0=R.arm_q(); p0=R.tcp()[0]
s=rb.ik_free(R,p0+[0,0,0.06],qh,seed=q0,attempts=2); R.move(s,2.0)
print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
A=R.cloud("agentview"); R.snap("agentview","/workspace/agent10.png")
m=(A[...,2]>0.905)&(A[...,2]<1.2)&(A[...,0]>-0.08)&(A[...,0]<0.10)&(A[...,1]>0.09)&(A[...,1]<0.25)
pts=A[m]; print("mug pts",len(pts),"z range",np.round([pts[:,2].min(),pts[:,2].max()],3),"rim-ish center",np.round(pts[pts[:,2]>pts[:,2].max()-0.02][:,:2].mean(0),4))
EOF

# openrua op 56
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(45); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
W=[[0.007,0.067,1.20],[-0.08,-0.25,1.20],[-0.17,-0.53,1.20],[-0.17,-0.53,1.005]]
q=R.arm_q(); sols=[]
for w in W:
    s=rb.ik_free(R,w,qh,seed=q,attempts=3)
    if s is None and w[1]==-0.53:
        w=[w[0],-0.50,w[2]]; s=rb.ik_free(R,w,qh,seed=q,attempts=3); print("fallback y=-0.50")
    assert s is not None, w
    sols.append(s); print("wp",w,"dq",np.round(np.abs(np.array(s)-np.array(q)).max(),2)); q=s
for i,s in enumerate(sols):
    R.move(s,4.0); print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
R.snap("agentview","/workspace/agent11.png"); R.snap("robot0_eye_in_hand","/workspace/eih7.png")
EOF

# openrua op 57
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(45); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
q=R.arm_q()
for y in (-0.44,-0.46,-0.48,-0.50,-0.52):
    row=[]
    for z in (1.005,1.05,1.10,1.15,1.20):
        s=rb.ik_free(R,[-0.17,y,z],qh,seed=q,attempts=3)
        row.append("ok " if s is not None else "-- ")
    print(y,"".join(row))
EOF

# openrua op 58
timeout 600 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(45); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
q=R.arm_q()
for y in (-0.48,-0.50):
    for z in (1.005,1.10):
        try: s=R.ik([-0.17,y,z],qh,seed=q,attempts=4); print(y,z,"plain IK ok",np.round(s,2), "valid:",rb.state_valid(R,s))
        except Exception as e: print(y,z,"plain IK fail")
EOF

# openrua op 59
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q()
for deg in (45,35,30,25):
    th=np.deg2rad(deg); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
    for x in (-0.17,-0.19,-0.21):
        row=[]
        for y in (-0.48,-0.49,-0.50,-0.51):
            ok=[]
            for z in (1.005,1.12):
                try: s=R.ik([x,y,z],qh,seed=q,attempts=6); ok.append("F" if rb.state_valid(R,s)[0] else "c")
                except Exception: ok.append("-")
            row.append("".join(ok))
        print(f"tilt{deg} x{x}: "+"  ".join(row))
EOF

# openrua op 60
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q()
th=np.deg2rad(35); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
for y_m in (-0.27,-0.28,-0.29,-0.30):
    s=rb.ik_free(R,[-0.17,y_m-0.083,1.005],qh,seed=q,attempts=4); print("insert y_m",y_m,"ok" if s is not None else "BLOCKED")
for z in (1.13,1.15,1.17,1.19):
    for y in (-0.47,-0.50):
        s=rb.ik_free(R,[-0.17,y,z],qh,seed=q,attempts=6); print("front y",y,"z",z,"ok" if s is not None else "BLOCKED")
# grasp + pre at 35 deg
bar=np.array([0.007,0.067,0.955])
print("grasp35",rb.ik_free(R,bar,qh,seed=q,attempts=4) is not None,"pre35",rb.ik_free(R,bar-0.08*hz,qh,seed=q,attempts=4) is not None)
EOF

# openrua op 61
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=R.arm_q()
for deg in (30,25,20):
    th=np.deg2rad(deg); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
    ins=[ "ok" if rb.ik_free(R,[-0.17,y_m-0.083,1.005],qh,seed=q,attempts=4) is not None else "X" for y_m in (-0.27,-0.28,-0.29,-0.30)]
    fr=[ f"{y}/{z}:"+("ok" if rb.ik_free(R,[-0.17,y,z],qh,seed=q,attempts=6) is not None else "X") for y in (-0.50,-0.51,-0.52) for z in (1.005,1.17)]
    bar=np.array([0.007,0.067,0.955])
    g=rb.ik_free(R,bar,qh,seed=q,attempts=4) is not None; p=rb.ik_free(R,bar-0.08*hz,qh,seed=q,attempts=4) is not None
    print(f"tilt{deg}: insert(-.27..-.30)={ins} front={fr} grasp={g} pre={p}")
EOF

# openrua op 62
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th45=np.deg2rad(45); hz45=np.array([0,np.sin(th45),-np.cos(th45)]); q45=rb.frame_quat(hz45,[1,0,0])
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
q=R.arm_q(); p=R.tcp()[0]
s=R.ik([p[0],p[1],0.9555],q45,seed=q,attempts=1); R.move(s,2.0); print("tcp",np.round(R.tcp()[0],4))
R.grip(0.04)
s=R.ik(np.array([p[0],p[1],0.9555])-0.08*hz45,q45,seed=R.arm_q(),attempts=1); R.move(s,2.0); print("tcp",np.round(R.tcp()[0],4))
bar=np.array([0.007,0.067,0.955]); pre=bar-0.08*hz
s=rb.ik_free(R,pre+[0,0,0.06],qh,seed=R.arm_q(),attempts=4); R.move(s,3.0); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih8_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih8.png")
m=(E[...,2]>0.935)&(E[...,2]<0.975)&(np.abs(E[...,0])<0.06)&(E[...,1]>0.03)&(E[...,1]<0.095)
pts=E[m]; print("bar pts",len(pts))
if len(pts): print("bar x",np.round([pts[:,0].min(),pts[:,0].max()],4),"y",np.round([pts[:,1].min(),pts[:,1].max()],4),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
m=(E[...,2]>0.99)&(E[...,2]<1.02)&(np.abs(E[...,0])<0.15)&(E[...,1]>0.05)&(E[...,1]<0.25)
pts=E[m][:,:2]
if len(pts)>20:
    x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
    print("rim center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4),"n",len(pts))
EOF

# openrua op 63
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
bar=np.array([0.0055,0.063,0.955]); pre=bar-0.08*hz
s_pre=rb.ik_free(R,pre,qh,seed=R.arm_q(),attempts=3); R.move(s_pre,2.0); print("tcp pre",np.round(R.tcp()[0],4))
qs=[]; seed=s_pre
for f in (0.25,0.5,0.75,1.0):
    s=R.ik(pre+f*(bar-pre),qh,seed=seed,attempts=1); qs.append(s); seed=s
R.move_path(qs,0.8)
q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
if err>0.01: R.move(qs[-1],2.0)
print("tcp grasp",np.round(R.tcp()[0],4))
gap=R.grip(0.0)
s=rb.ik_free(R,bar+[0,0,0.06],qh,seed=R.arm_q(),attempts=2); R.move(s,2.0)
print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
A=R.cloud("agentview"); R.snap("agentview","/workspace/agent12.png")
m=(A[...,2]>0.905)&(A[...,2]<1.2)&(A[...,0]>-0.08)&(A[...,0]<0.10)&(A[...,1]>0.085)&(A[...,1]<0.25)
pts=A[m]; print("mug pts",len(pts),"z range",np.round([pts[:,2].min(),pts[:,2].max()],3))
top=pts[pts[:,2]>pts[:,2].max()-0.015]; print("rim z spread (level check): min",round(top[:,2].min(),3),"max",round(top[:,2].max(),3),"center",np.round(top[:,:2].mean(0),4))
EOF

# openrua op 64
timeout 1700 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
W=[[0.0058,0.063,1.20],[-0.08,-0.25,1.20],[-0.17,-0.42,1.20],[-0.17,-0.51,1.17],[-0.17,-0.51,1.005]]
q=R.arm_q(); sols=[]
for w in W:
    s=rb.ik_free(R,w,qh,seed=q,attempts=4); assert s is not None, w
    print("wp",w,"dq",round(float(np.abs(np.array(s)-np.array(q)).max()),2)); sols.append(s); q=s
for w,s in zip(W,sols):
    R.move(s,4.0); print("  tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
R.snap("agentview","/workspace/agent13.png"); R.snap("robot0_eye_in_hand","/workspace/eih9.png")
EOF

# openrua op 65
timeout 300 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
q=np.array(R.arm_q()); print("q",np.round(q,3))
p,qq=R.tcp(); print("tcp",np.round(p,4)); Rm=rb.quat_to_R(qq); print("hand z",np.round(Rm[:,2],3),"hand y",np.round(Rm[:,1],3))
print("gap",round(R.finger_gap(),4))
print("valid",rb.state_valid(R,q))
R.snap("agentview","/workspace/agent13.png"); R.snap("birdview","/workspace/bird6.png")
EOF

# openrua op 66
cat >> /workspace/rb.py <<'EOF'


def attach_box(R, cid, size, pose_in_hand, link="panda_hand", remove=False):
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import ApplyPlanningScene
    aco = AttachedCollisionObject(); aco.link_name = link
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link8", "panda_link7"]
    co = aco.object; co.header.frame_id = link; co.id = cid
    if remove:
        co.operation = CollisionObject.REMOVE
    else:
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size]))
        p = Pose(); p.position.x, p.position.y, p.position.z = map(float, pose_in_hand); p.orientation.w = 1.0
        co.primitive_poses.append(p); co.operation = CollisionObject.ADD
    cli = R.node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(10)
    req = ApplyPlanningScene.Request(); req.scene.is_diff = True
    req.scene.robot_state.is_diff = True
    req.scene.robot_state.attached_collision_objects.append(aco)
    if remove:
        # also drop it from the world in case it was detached there
        w = CollisionObject(); w.id = cid; w.header.frame_id = "world"; w.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects.append(w)
    fut = cli.call_async(req); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=30)
    return fut.result().success


def plan_to_joints(R, q_goal, secs=10.0, attempts=5, vel=0.3):
    """MoveIt plan-only to a joint goal from the current state. Returns list of (t, positions)."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    if not hasattr(R, "_mg"):
        R._mg = ActionClient(R.node, MoveGroup, M["planning"]["move_action"]); R._mg.wait_for_server(10)
    g = MoveGroup.Goal(); r = g.request
    r.group_name = M["planning"]["group"]; r.allowed_planning_time = float(secs)
    r.num_planning_attempts = int(attempts); r.max_velocity_scaling_factor = float(vel)
    r.max_acceleration_scaling_factor = float(vel)
    r.start_state.is_diff = True
    c = Constraints()
    for n, v in zip(ARM, q_goal):
        c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v),
                                                   tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
    r.goal_constraints.append(c)
    g.planning_options.plan_only = True
    fut = R._mg.send_goal_async(g); rclpy.spin_until_future_complete(R.node, fut, timeout_sec=60)
    rf = fut.result().get_result_async(); rclpy.spin_until_future_complete(R.node, rf, timeout_sec=120)
    res = rf.result().result
    if res.error_code.val != 1:
        print("plan failed", res.error_code.val); return None
    traj = res.planned_trajectory.joint_trajectory
    idx = [traj.joint_names.index(n) for n in ARM]
    return [(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9, [pt.positions[i] for i in idx])
            for pt in traj.points]


def exec_plan(R, plan, tscale=1.5, min_dt=0.3):
    """Execute a planned trajectory (sparsified) with the FJT client, then converge on the final point."""
    pts = [p for p in plan]
    goal_qs = []; last_t = -1
    for t, q in pts:
        if t - last_t >= min_dt or (t, q) == pts[-1]:
            goal_qs.append((t, q)); last_t = t
    from control_msgs.action import FollowJointTrajectory
    gl = FollowJointTrajectory.Goal(); gl.trajectory.joint_names = list(ARM)
    for t, q in goal_qs:
        tt = max(0.5, t * tscale)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9)); gl.trajectory.points.append(pt)
    send = R.fjt.send_goal_async(gl); rclpy.spin_until_future_complete(R.node, send, timeout_sec=60)
    res = send.result().get_result_async(); rclpy.spin_until_future_complete(R.node, res, timeout_sec=600)
    qf = np.array(goal_qs[-1][1]); err = np.abs(np.array(R.arm_q()) - qf).max()
    print(f"exec_plan: {len(goal_qs)} pts, code={res.result().result.error_code}, err={err:.4f}")
    if err > 0.01:
        R.move(list(qf), 3.0)
    return np.abs(np.array(R.arm_q()) - qf).max()
EOF
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
# mug in hand frame: computed at grasp (30deg): center = TCP + 0.0918*hx + 0.001*hz -> hand frame (0.092, 0, 0.1034+0.001)
print("attach",rb.attach_box(R,"held_mug",(0.10,0.10,0.11),(0.092,0.0,0.105)))
q=np.array(R.arm_q()); print("valid with mug",rb.state_valid(R,q))
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
seed=[0.153,0.891,-0.281,-1.567,1.165,1.98,-1.581]
W3=[-0.17,-0.42,1.20]
best=None
for k in range(8):
    s=rb.ik_free(R,W3,qh,seed=list(np.array(seed)+np.random.default_rng(k).normal(0,0.2,7)),attempts=1)
    if s is not None:
        d=np.abs(np.array(s)-np.array(seed)).max()
        if best is None or d<best[0]: best=(d,s)
print("W3 sol dq-from-family",None if best is None else (round(best[0],2),np.round(best[1],3)))
np.save("/workspace/w3_q.npy",np.array(best[1]))
plan=rb.plan_to_joints(R,best[1],secs=15.0,attempts=10)
print("plan pts",None if plan is None else len(plan), "T", None if plan is None else round(plan[-1][0],2))
if plan: np.save("/workspace/plan_recover.npy",np.array([[t]+list(q) for t,q in plan]))
EOF

# openrua op 67
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
P=np.load("/workspace/plan_recover.npy"); plan=[(r[0],list(r[1:])) for r in P]
err=rb.exec_plan(R,plan,tscale=3.0,min_dt=0.2)
print("final err",round(err,4),"tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
p,qq=R.tcp(); Rm=rb.quat_to_R(qq); print("hand z",np.round(Rm[:,2],3),"hand y",np.round(Rm[:,1],3))
R.snap("agentview","/workspace/agent14.png")
EOF

# openrua op 68
cat >> /workspace/rb.py <<'EOF'


def cart_path(R, targets, quat, step=0.02, maxdq=0.5, seed=None):
    """Straight-line TCP waypoints (world) -> continuous joint path, collision-validated. None on failure."""
    q = list(R.arm_q() if seed is None else seed)
    p0 = np.array(R.tcp(q)[0]); qs = []
    for tgt in targets:
        tgt = np.array(tgt, float); n = max(1, int(np.ceil(np.linalg.norm(tgt - p0) / step)))
        for i in range(1, n + 1):
            wp = p0 + (tgt - p0) * i / n
            try:
                s = R.ik(wp, quat, seed=q, attempts=1)
            except RuntimeError:
                print("cart_path: IK fail at", np.round(wp, 3)); return None
            dq = np.abs(np.array(s) - np.array(q)).max()
            if dq > maxdq:
                print("cart_path: branch jump", round(dq, 2), "at", np.round(wp, 3)); return None
            v = state_valid(R, s)
            if v is not None and not v[0]:
                print("cart_path: collision at", np.round(wp, 3), v[1]); return None
            qs.append(s); q = s
        p0 = tgt
    return qs
EOF
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
qs=rb.cart_path(R,[[-0.17,-0.51,1.17],[-0.17,-0.51,1.005]],qh,step=0.02)
print("path pts",None if qs is None else len(qs))
if qs:
    R.move_path(qs,0.6)
    q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
    if err>0.01: R.move(qs[-1],2.0)
    print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
    R.snap("agentview","/workspace/agent15.png"); R.snap("robot0_eye_in_hand","/workspace/eih10.png")
EOF

# openrua op 69
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
q0=R.arm_q()
for z in (1.20,1.17,1.13):
    q=list(q0); row=[]
    for y in (-0.44,-0.46,-0.48,-0.49,-0.50,-0.51):
        try:
            s=R.ik([-0.17,y,z],qh,seed=q,attempts=1); dq=np.abs(np.array(s)-np.array(q)).max(); v=rb.state_valid(R,s)[0]
            row.append(f"{y}:{dq:.2f}{'V' if v else 'c'}"); 
            if dq<0.5: q=s
        except RuntimeError: row.append(f"{y}:--")
    print("z",z," ".join(row))
EOF

# openrua op 70
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
qs=rb.cart_path(R,[[-0.17,-0.50,1.17],[-0.17,-0.51,1.13],[-0.17,-0.51,1.005]],qh,step=0.02)
print("path pts",None if qs is None else len(qs))
if qs:
    R.move_path(qs,0.6)
    q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
    if err>0.01: R.move(qs[-1],2.0)
    print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
    R.snap("agentview","/workspace/agent15.png"); R.snap("robot0_eye_in_hand","/workspace/eih10.png")
EOF

# openrua op 71
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
qs=rb.cart_path(R,[[-0.17,-0.363,1.005]],qh,step=0.02)
print("path pts",None if qs is None else len(qs))
if qs:
    R.move_path(qs,0.6)
    q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
    if err>0.01: R.move(qs[-1],2.0)
    print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
    R.snap("agentview","/workspace/agent16.png"); R.snap("robot0_eye_in_hand","/workspace/eih11.png"); R.snap("birdview","/workspace/bird7.png")
EOF

# openrua op 72
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
print("valid now",rb.state_valid(R,R.arm_q()))
p=R.tcp()[0]
qs=rb.cart_path(R,[[-0.17,-0.51,p[2]]],qh,step=0.02)
print("pts",None if qs is None else len(qs))
if qs:
    R.move_path(qs,0.6); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
    if err>0.01: R.move(qs[-1],2.0)
print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
R.snap("robot0_eye_in_hand","/workspace/eih12.png"); R.snap("agentview","/workspace/agent17.png")
A=R.cloud("agentview"); np.save("/workspace/agent17_pc.npy",A)
# mug points: near x -0.17, y -0.48..-0.37, z 0.93..1.1
m=(A[...,0]>-0.25)&(A[...,0]<-0.09)&(A[...,1]>-0.50)&(A[...,1]<-0.365)&(A[...,2]>0.90)&(A[...,2]<1.12)
pts=A[m]; print("mug pts",len(pts))
if len(pts):
    print("z range",np.round([pts[:,2].min(),pts[:,2].max()],3))
    for z0 in (0.93,0.95,0.97,0.99,1.01,1.03,1.05):
        s=pts[(pts[:,2]>z0)&(pts[:,2]<z0+0.02)]
        if len(s): print(f"  z{z0}: n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 73
timeout 300 python3 -u - <<'EOF'
import numpy as np
E=np.load("/workspace/eih2_pc.npy")
m=(E[...,0]>-0.24)&(E[...,0]<-0.10)&(E[...,1]>-0.36)&(E[...,1]<-0.17)&(E[...,2]>0.90)&(E[...,2]<1.0)&np.isfinite(E[...,2])
pts=E[m]; print("cavity-floor-ish pts",len(pts))
for y0 in np.arange(-0.36,-0.17,0.01):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(s): print(f"y {y0:.2f}: n={len(s):4d} z min {s[:,2].min():.3f} med {np.median(s[:,2]):.3f} max {s[:,2].max():.3f}")
# x profile of the floor at mid depth
s=pts[(pts[:,1]>-0.30)&(pts[:,1]<-0.25)]
for x0 in np.arange(-0.24,-0.10,0.02):
    t=s[(s[:,0]>=x0)&(s[:,0]<x0+0.02)]
    if len(t): print(f"x {x0:.2f}: n={len(t)} z med {np.median(t[:,2]):.3f}")
# ceiling
m=(E[...,0]>-0.24)&(E[...,0]<-0.10)&(E[...,1]>-0.36)&(E[...,1]<-0.17)&(E[...,2]>1.04)&(E[...,2]<1.12)
c=E[m]; print("ceiling pts",len(c), "z min/med",np.round([c[:,2].min(),np.median(c[:,2])],3) if len(c) else "")
EOF

# openrua op 74
timeout 300 python3 -u - <<'EOF'
import numpy as np
E=np.load("/workspace/eih2_pc.npy")
# points on/near the front face plane
m=(E[...,1]>-0.375)&(E[...,1]<-0.345)&(E[...,2]>0.95)&(E[...,2]<1.08)&(E[...,0]>-0.35)&(E[...,0]<0.1)&np.isfinite(E[...,2])
f=E[m]; print("face-plane pts",len(f))
for x0 in np.arange(-0.34,0.08,0.02):
    s=f[(f[:,0]>=x0)&(f[:,0]<x0+0.02)]
    print(f"x {x0:.2f}: n={len(s):4d}", "" if not len(s) else f"y med {np.median(s[:,1]):.3f} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
# cavity floor x extent at depth y -0.30..-0.25 (no x mask)
m=(E[...,1]>-0.30)&(E[...,1]<-0.25)&(E[...,2]>0.935)&(E[...,2]<0.955)&np.isfinite(E[...,2])
fl=E[m]; print("floor x range",np.round([fl[:,0].min(),fl[:,0].max()],3),"n",len(fl))
# side walls: points at y -0.30..-0.25, z 0.96..1.06
m=(E[...,1]>-0.30)&(E[...,1]<-0.25)&(E[...,2]>0.96)&(E[...,2]<1.06)&np.isfinite(E[...,2])
w=E[m]; print("wall pts x hist:",np.histogram(w[:,0],bins=np.arange(-0.34,0.08,0.02))[0])
EOF

# openrua op 75
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
p=R.tcp()[0]
qs=rb.cart_path(R,[[p[0],p[1],0.962]],qh,step=0.02)
R.move_path(qs,0.5); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
R.grip(0.04)
p=R.tcp()[0]
qs=rb.cart_path(R,[[p[0],p[1],p[2]+0.07]],qh,step=0.02); R.move_path(qs,0.5); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih13_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih13.png")
# rim + handle bar
m=(E[...,2]>0.99)&(E[...,2]<1.02)&(E[...,0]>-0.3)&(E[...,0]<0.0)&(E[...,1]>-0.6)&(E[...,1]<-0.36)&np.isfinite(E[...,2])
pts=E[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
print("rim n",len(pts),"center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4))
mz=E[...,2]; m=(mz>0.935)&(mz<0.975)&(np.abs(E[...,0]-c[0])<0.06)&(E[...,1]<c[1]-0.055)&(E[...,1]>c[1]-0.11)&np.isfinite(mz)
bar=E[m]; print("bar n",len(bar))
if len(bar): print("bar x",np.round([bar[:,0].min(),bar[:,0].max()],4),"y",np.round([bar[:,1].min(),bar[:,1].max()],4),"z",np.round([bar[:,2].min(),bar[:,2].max()],3))
m=(mz>0.90)&(mz<1.02)&(np.abs(E[...,0]-c[0])<0.06)&(np.abs(E[...,1]-c[1])<0.06)&np.isfinite(mz); body=E[m]
print("mug top z",round(body[:,2].max(),3))
EOF

# openrua op 76
timeout 300 python3 -u - <<'EOF'
import numpy as np
E=np.load("/workspace/eih13_pc.npy"); z=E[...,2]
box=(E[...,0]>-0.25)&(E[...,0]<-0.09)&(E[...,1]>-0.55)&(E[...,1]<-0.37)&np.isfinite(z)
m=box&(z>0.99)&(z<1.01)
pts=E[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
r=float(np.sqrt(c[2]+c[0]**2+c[1]**2)); print("rim n",len(pts),"center",np.round(c[:2],4),"r",round(r,4))
# robust: keep points within 8mm of fitted circle and refit
d=np.abs(np.linalg.norm(pts-c[:2],axis=1)-r); pts2=pts[d<0.008]
x,y=pts2[:,0],pts2[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]; r=float(np.sqrt(c[2]+c[0]**2+c[1]**2))
print("refit n",len(pts2),"center",np.round(c[:2],4),"r",round(r,4))
cx,cy=c[0],c[1]
m=box&(z>0.925)&(z<0.995); pts=E[m]; d=np.linalg.norm(pts[:,:2]-[cx,cy],axis=1); h=pts[d>0.056]
print("handle n",len(h),"x",np.round([h[:,0].min(),h[:,0].max()],4),"y",np.round([h[:,1].min(),h[:,1].max()],4),"z",np.round([h[:,2].min(),h[:,2].max()],3))
for z0,z1 in ((0.935,0.975),(0.975,0.995)):
    bb=h[(h[:,2]>z0)&(h[:,2]<z1)]
    if len(bb): print(f"  z{z0}-{z1}: n={len(bb)} x[{bb[:,0].min():.4f},{bb[:,0].max():.4f}] y[{bb[:,1].min():.4f},{bb[:,1].max():.4f}]")
print("mug top z",round(E[box&(z<1.03)][:,2].max(),3))
EOF

# openrua op 77
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
cx,cy=-0.1721,-0.4643
gx=cx+0.0494
q=R.arm_q()
s_hi=rb.ik_free(R,[gx,cy,1.06],qv,seed=q,attempts=6); assert s_hi is not None
print("dq to rim-pre",round(float(np.abs(np.array(s_hi)-np.array(q)).max()),2))
R.move(s_hi,4.0); print("tcp",np.round(R.tcp()[0],4))
qs=rb.cart_path(R,[[gx,cy,0.975]],qv,step=0.02); assert qs
R.move_path(qs,0.5); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0)
gap=R.grip(0.0)
if gap<0.004 or gap>0.03: print("BAD GRASP gap",gap); raise SystemExit
qs=rb.cart_path(R,[[gx,cy,1.01],[gx,-0.42,1.01],[gx,-0.42,0.98]],qv,step=0.02); assert qs
R.move_path(qs,0.5); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
if err>0.01: R.move(qs[-1],2.0)
R.grip(0.04)
qs=rb.cart_path(R,[[gx-0.05,-0.42,1.09]],qv,step=0.02); R.move_path(qs,0.5); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih14_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih14.png")
z=E[...,2]; box=(E[...,0]>-0.25)&(E[...,0]<-0.09)&(E[...,1]>-0.56)&(E[...,1]<-0.37)&np.isfinite(z)
m=box&(z>0.99)&(z<1.01); pts=E[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
print("rim n",len(pts),"center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4))
m=box&(z>0.925)&(z<0.995); pts=E[m]; d=np.linalg.norm(pts[:,:2]-c[:2],axis=1); h=pts[d>0.056]
print("handle n",len(h))
for z0,z1 in ((0.935,0.975),(0.975,0.995)):
    bb=h[(h[:,2]>z0)&(h[:,2]<z1)]
    if len(bb): print(f"  z{z0}-{z1}: n={len(bb)} x[{bb[:,0].min():.4f},{bb[:,0].max():.4f}] y[{bb[:,1].min():.4f},{bb[:,1].max():.4f}]")
print("mug top z",round(E[box&(z<1.03)][:,2].max(),3))
EOF

# openrua op 78
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
qv=rb.frame_quat([0,0,-1],[1,0,0])
s=rb.ik_free(R,[-0.16,-0.49,1.12],qv,seed=R.arm_q(),attempts=4)
if s is None: s=rb.ik_free(R,[-0.16,-0.47,1.12],qv,seed=R.arm_q(),attempts=4)
print("dq",round(float(np.abs(np.array(s)-np.array(R.arm_q())).max()),2)); R.move(s,3.0); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih15_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih15.png")
z=E[...,2]; box=(E[...,0]>-0.25)&(E[...,0]<-0.07)&(E[...,1]>-0.58)&(E[...,1]<-0.36)&np.isfinite(z)
m=box&(z>0.99)&(z<1.01); pts=E[m][:,:2]; x,y=pts[:,0],pts[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
print("rim n",len(pts),"center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4))
m=box&(z>0.925)&(z<0.995); pts=E[m]; d=np.linalg.norm(pts[:,:2]-c[:2],axis=1); h=pts[d>0.056]
print("handle n",len(h))
if len(h):
    print("handle x",np.round([h[:,0].min(),h[:,0].max()],4),"y",np.round([h[:,1].min(),h[:,1].max()],4),"z",np.round([h[:,2].min(),h[:,2].max()],3))
    for z0,z1 in ((0.935,0.975),(0.975,0.995)):
        bb=h[(h[:,2]>z0)&(h[:,2]<z1)]
        if len(bb): print(f"  z{z0}-{z1}: n={len(bb)} x[{bb[:,0].min():.4f},{bb[:,0].max():.4f}] xmean {bb[:,0].mean():.4f} y[{bb[:,1].min():.4f},{bb[:,1].max():.4f}]")
EOF

# openrua op 79
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
from scipy.spatial.transform import Rotation as Rot
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); hy=np.array([1.,0,0]); hx=np.cross(hy,hz); qh=rb.frame_quat(hz,hy)
bar=np.array([-0.1585,-0.499,0.955]); pre=bar+[0,0,0.05]
q=R.arm_q()
s_pre=rb.ik_free(R,pre,qh,seed=q,attempts=4); assert s_pre is not None
print("dq",round(float(np.abs(np.array(s_pre)-np.array(q)).max()),2)); R.move(s_pre,3.0); print("tcp pre",np.round(R.tcp()[0],4))
R.snap("robot0_eye_in_hand","/workspace/eih16.png")
qs=rb.cart_path(R,[bar],qh,step=0.0125); assert qs
R.move_path(qs,0.5); qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("err",round(err,4))
if err>0.01: R.move(qs[-1],2.0)
print("tcp grasp",np.round(R.tcp()[0],4))
gap=R.grip(0.0); print("gap",round(gap,4))
# attach mug box with world-aligned orientation: center = TCP + (0,0.080,-0.005)
o=np.array([0,0.080,-0.005]); Rh=np.stack([hx,hy,hz],1)
c_h=Rh.T@o+np.array([0,0,0.1034]); q_h=Rot.from_matrix(Rh.T).as_quat()
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
print("detach old",rb.attach_box(R,"held_mug",(0.1,0.1,0.1),(0,0,0),remove=True))
aco=AttachedCollisionObject(); aco.link_name="panda_hand"; aco.touch_links=["panda_hand","panda_leftfinger","panda_rightfinger"]
co=aco.object; co.header.frame_id="panda_hand"; co.id="held_mug"
co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[0.10,0.10,0.10]))
p=Pose(); p.position.x,p.position.y,p.position.z=map(float,c_h); p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q_h)
co.primitive_poses.append(p); co.operation=CollisionObject.ADD
cli=R.node.create_client(ApplyPlanningScene,"/apply_planning_scene"); cli.wait_for_service(10)
req=ApplyPlanningScene.Request(); req.scene.is_diff=True; req.scene.robot_state.is_diff=True; req.scene.robot_state.attached_collision_objects.append(aco)
fut=cli.call_async(req); rb.rclpy.spin_until_future_complete(R.node,fut,timeout_sec=30); print("attach",fut.result().success)
print("hand-frame box center",np.round(c_h,4),"valid",rb.state_valid(R,R.arm_q()))
EOF

# openrua op 80
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
qs=rb.cart_path(R,[[-0.159,-0.499,1.019]],qh,step=0.02); assert qs
R.move_path(qs,0.5); print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
A=R.cloud("agentview"); R.snap("agentview","/workspace/agent18.png")
m=(A[...,0]>-0.23)&(A[...,0]<-0.09)&(A[...,1]>-0.48)&(A[...,1]<-0.36)&(A[...,2]>0.93)&(A[...,2]<1.12)&np.isfinite(A[...,2])
pts=A[m]; print("mug pts",len(pts),"z range",np.round([pts[:,2].min(),pts[:,2].max()],3))
top=pts[pts[:,2]>pts[:,2].max()-0.01]; print("rim y range",np.round([top[:,1].min(),top[:,1].max()],3),"x range",np.round([top[:,0].min(),top[:,0].max()],3))
# lowest visible body points -> bottom estimate
low=pts[pts[:,2]<pts[:,2].min()+0.01]; print("low pts y",np.round([low[:,1].min(),low[:,1].max()],3))
EOF

# openrua op 81
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
qs=rb.cart_path(R,[[-0.159,-0.36,1.019]],qh,step=0.02)
print("pts",None if qs is None else len(qs))
if qs:
    R.move_path(qs,0.6); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4))
    if err>0.01: R.move(qs[-1],2.0,retries=1)
    print("tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
    R.snap("robot0_eye_in_hand","/workspace/eih17.png"); R.snap("agentview","/workspace/agent19.png")
EOF

# openrua op 82
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
p=R.tcp()[0]
qs=rb.cart_path(R,[[p[0],p[1],1.000]],qh,step=0.01); assert qs
R.move_path(qs,0.5); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0,retries=1); print("tcp",np.round(R.tcp()[0],4))
R.grip(0.04)
print("detach",rb.attach_box(R,"held_mug",(0.1,0.1,0.1),(0,0,0),remove=True))
p=R.tcp()[0]
qs=rb.cart_path(R,[[p[0],p[1],1.02],[p[0],-0.50,1.02],[p[0],-0.50,1.13]],qh,step=0.02); assert qs
R.move_path(qs,0.5); q=np.array(R.arm_q()); err=np.abs(q-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0)
R.snap("robot0_eye_in_hand","/workspace/eih18.png"); R.snap("agentview","/workspace/agent20.png")
EOF

# openrua op 83
timeout 600 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
# look into the cavity with the wrist camera: tilt hand so camera looks along +y and down into the opening
th=np.deg2rad(55); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
s=rb.ik_free(R,[-0.16,-0.50,1.10],qh,seed=R.arm_q(),attempts=4)
if s is not None and np.abs(np.array(s)-np.array(R.arm_q())).max()<1.0:
    R.move(s,3.0); print("tcp",np.round(R.tcp()[0],3))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih19_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih19.png")
z=E[...,2]; box=(E[...,0]>-0.25)&(E[...,0]<-0.07)&(E[...,1]>-0.35)&(E[...,1]<-0.19)&np.isfinite(z)
m=box&(z>1.03)&(z<1.07); pts=E[m]
print("rim-height pts in cavity",len(pts))
if len(pts)>20:
    xy=pts[:,:2]; x,y=xy[:,0],xy[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
    print("rim center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
allm=box&(z>0.95)&(z<1.08); a=E[allm]; print("mug body pts",len(a),"y range",np.round([a[:,1].min(),a[:,1].max()],3),"x range",np.round([a[:,0].min(),a[:,0].max()],3),"z max",round(a[:,2].max(),3))
EOF

# openrua op 84
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
R.grip(0.0)
q=R.arm_q()
s=rb.ik_free(R,[-0.157,-0.43,1.02],qh,seed=q,attempts=3); assert s is not None and np.abs(np.array(s)-np.array(q)).max()<1.0
R.move(s,3.0); print("tcp",np.round(R.tcp()[0],4))
qs=rb.cart_path(R,[[-0.157,-0.43,0.965],[-0.157,-0.34,0.965]],qh,step=0.015); assert qs
R.move_path(qs,0.5); qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0,retries=1); print("tcp",np.round(R.tcp()[0],4))
# back off and up
qs=rb.cart_path(R,[[-0.157,-0.43,0.98],[-0.157,-0.50,1.13]],qh,step=0.02); assert qs
R.move_path(qs,0.5); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih20_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih20.png")
z=E[...,2]; box=(E[...,0]>-0.25)&(E[...,0]<-0.07)&(E[...,1]>-0.40)&(E[...,1]<-0.19)&np.isfinite(z)
a=E[box&(z>0.95)&(z<1.08)]; print("mug/handle pts",len(a),"y min (handle end)",round(a[:,1].min(),4),"y max",round(a[:,1].max(),4),"x range",np.round([a[:,0].min(),a[:,0].max()],3),"z max",round(a[:,2].max(),3))
m=box&(z>1.03)&(z<1.06); pts=E[m]
if len(pts)>20:
    xy=pts[:,:2]; x,y=xy[:,0],xy[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
    print("rim center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4))
EOF

# openrua op 85
python3 - <<'EOF'
import re
s=open("rb.py").read()
s=s.replace("def cart_path(R, targets, quat, step=0.02, maxdq=0.5, seed=None):","def cart_path(R, targets, quat, step=0.02, maxdq=0.5, seed=None, validate=True):")
s=s.replace("            v = state_valid(R, s)\n            if v is not None and not v[0]:","            v = state_valid(R, s) if validate else None\n            if v is not None and not v[0]:")
open("rb.py","w").write(s)
EOF
timeout 1500 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
th=np.deg2rad(30); hz=np.array([0,np.sin(th),-np.cos(th)]); qh=rb.frame_quat(hz,[1,0,0])
print("gap",round(R.finger_gap(),4),"tcp",np.round(R.tcp()[0],4))
qs=rb.cart_path(R,[[-0.157,-0.43,0.965],[-0.157,-0.36,0.965]],qh,step=0.015); assert qs
R.move_path(qs,0.5); qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0,retries=1)
qs=rb.cart_path(R,[[-0.157,-0.34,0.965]],qh,step=0.01,validate=False); assert qs
R.move_path(qs,0.5); qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("err",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01: R.move(qs[-1],2.0,retries=1); print("tcp",np.round(R.tcp()[0],4))
qs=rb.cart_path(R,[[-0.157,-0.43,0.98],[-0.157,-0.50,1.13]],qh,step=0.02,validate=False); assert qs
R.move_path(qs,0.5); print("tcp",np.round(R.tcp()[0],4))
E=R.cloud("robot0_eye_in_hand"); np.save("/workspace/eih20_pc.npy",E); R.snap("robot0_eye_in_hand","/workspace/eih20.png")
z=E[...,2]; box=(E[...,0]>-0.25)&(E[...,0]<-0.07)&(E[...,1]>-0.40)&(E[...,1]<-0.19)&np.isfinite(z)
a=E[box&(z>0.95)&(z<1.08)]; print("mug/handle pts",len(a),"y min (handle end)",round(a[:,1].min(),4),"y max",round(a[:,1].max(),4),"x range",np.round([a[:,0].min(),a[:,0].max()],3),"z max",round(a[:,2].max(),3))
m=box&(z>1.03)&(z<1.06); pts=E[m]
if len(pts)>20:
    xy=pts[:,:2]; x,y=xy[:,0],xy[:,1]; Am=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2; c=np.linalg.lstsq(Am,b,rcond=None)[0]
    print("rim center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4))
EOF

# openrua op 86
grep -n "def cart_path" -A 40 /workspace/rb.py | head -60; grep -n "def frame_quat" -A 12 /workspace/rb.py; grep -n "def plan_to_joints\|def exec_plan\|def apply_scene\|def _box\|def ik_free\|def state_valid" /workspace/rb.py

# openrua op 87
cat >> /workspace/rb.py <<'EOF'


def pose_path(R, poses, maxdq=0.5, seed=None, validate=True, verbose=False):
    """List of (pos, quat) TCP poses -> continuous joint path (seeded IK per pose). None on failure."""
    q = list(R.arm_q() if seed is None else seed); qs = []
    for i, (p, quat) in enumerate(poses):
        try:
            s = R.ik(np.array(p, float), quat, seed=q, attempts=1)
        except RuntimeError:
            print("pose_path: IK fail at", i, np.round(p, 3)); return None
        dq = np.abs(np.array(s) - np.array(q)).max()
        if dq > maxdq:
            print("pose_path: branch jump", round(dq, 2), "at", i, np.round(p, 3)); return None
        v = state_valid(R, s) if validate else None
        if v is not None and not v[0]:
            print("pose_path: collision at", i, np.round(p, 3), v[1]); return None
        if verbose: print(i, np.round(p, 3), "dq", round(dq, 3))
        qs.append(s); q = s
    return qs
EOF
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
hinge=np.array([-0.30,-0.36]); r=0.16; off=0.02; z=1.02
def arc(theta, phis):
    th=np.deg2rad(theta); poses=[]
    for phi in phis:
        f=np.deg2rad(phi); n=np.array([-np.sin(f),np.cos(f),0]); rad=np.array([np.cos(f),np.sin(f),0])
        p=np.array([*hinge,z])+r*rad-off*n
        hz=np.sin(th)*n-np.cos(th)*np.array([0,0,1.0])
        poses.append((p, rb.frame_quat(hz, rad)))
    return poses
phis=np.arange(-112,4,4.0)
for theta in (45,60,35):
    poses=arc(theta,phis)
    # find a seed for the first pose with many attempts
    try: q0=R.ik(poses[0][0],poses[0][1],attempts=30)
    except RuntimeError: print(theta,"start IK fail"); continue
    v=rb.state_valid(R,q0); print("theta",theta,"start ok, valid",v[0], v[1] if not v[0] else "")
    qs=rb.pose_path(R,poses,seed=q0,validate=True)
    print("theta",theta,"path", "OK" if qs else "FAIL", "nq", len(qs) if qs else 0)
    if qs:
        Q=np.array(qs); print("joint ranges",np.round(Q.min(0),2),np.round(Q.max(0),2)); break
EOF

# openrua op 88
timeout 900 python3 -u - <<'EOF'
import rb, numpy as np
from moveit_msgs.msg import CollisionObject
R=rb.Robot()
co=CollisionObject(); co.id="door"; co.header.frame_id="world"; co.operation=CollisionObject.REMOVE
rb.apply_scene(R,[co]); print("door box removed")
hinge=np.array([-0.30,-0.36]); r=0.16; off=0.02; z=1.02
def arc(theta, phis):
    th=np.deg2rad(theta); poses=[]
    for phi in phis:
        f=np.deg2rad(phi); n=np.array([-np.sin(f),np.cos(f),0]); rad=np.array([np.cos(f),np.sin(f),0])
        p=np.array([*hinge,z])+r*rad-off*n
        hz=np.sin(th)*n-np.cos(th)*np.array([0,0,1.0])
        poses.append((p, rb.frame_quat(hz, rad)))
    return poses
phis=np.arange(-112,4,4.0)
for theta in (45,60,35):
    poses=arc(theta,phis)
    try: q0=R.ik(poses[0][0],poses[0][1],attempts=30)
    except RuntimeError: print(theta,"start IK fail"); continue
    v=rb.state_valid(R,q0); print("theta",theta,"start valid",v[0], v[1] if not v[0] else "")
    qs=rb.pose_path(R,poses,seed=q0,validate=True)
    print("theta",theta,"path", "OK" if qs else "FAIL", "nq", len(qs) if qs else 0)
    if qs:
        Q=np.array(qs); print("joint ranges",np.round(Q.min(0),2),np.round(Q.max(0),2)); np.save("/workspace/door_arc_q.npy",Q); print("saved theta",theta); break
EOF

# openrua op 89
timeout 1200 python3 -u - <<'EOF'
import rb, numpy as np
R=rb.Robot()
lim=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
hinge=np.array([-0.30,-0.36]); r=0.16; off=0.02; z=1.02
def arc(theta, phis):
    th=np.deg2rad(theta); poses=[]
    for phi in phis:
        f=np.deg2rad(phi); n=np.array([-np.sin(f),np.cos(f),0]); rad=np.array([np.cos(f),np.sin(f),0])
        p=np.array([*hinge,z])+r*rad-off*n
        hz=np.sin(th)*n-np.cos(th)*np.array([0,0,1.0])
        poses.append((p, rb.frame_quat(hz, rad)))
    return poses
phis=np.arange(-112,4,4.0); poses=arc(45,phis)
best=None
for k in range(12):
    try: q0=R.ik(poses[0][0],poses[0][1],seed=list(np.random.uniform(lim[:,0]*0.6,lim[:,1]*0.6)),attempts=5)
    except RuntimeError: continue
    if not rb.state_valid(R,q0)[0]: continue
    qs=rb.pose_path(R,poses,seed=q0,validate=True)
    if not qs: continue
    Q=np.array(qs); margin=np.minimum(Q-lim[:,0],lim[:,1]-Q).min()
    print(k,"margin",round(margin,3),"q0",np.round(Q[0],2))
    if best is None or margin>best[0]: best=(margin,Q)
print("best margin",round(best[0],3)); np.save("/workspace/door_arc_q.npy",best[1])
EOF

# openrua op 90
cat > /workspace/door_close.py <<'EOF'
import rb, numpy as np
R=rb.Robot()
Q=np.load("/workspace/door_arc_q.npy"); q0=list(Q[0])
hinge=np.array([-0.30,-0.36]); r=0.16; z=1.02; th=np.deg2rad(45)
def pose(phi, off):
    f=np.deg2rad(phi); n=np.array([-np.sin(f),np.cos(f),0]); rad=np.array([np.cos(f),np.sin(f),0])
    p=np.array([*hinge,z])+r*rad-off*n; hz=np.sin(th)*n-np.cos(th)*np.array([0,0,1.0])
    return p, rb.frame_quat(hz, rad)
print("start tcp",np.round(R.tcp()[0],4),"gap",round(R.finger_gap(),4))
# 1. re-add door box, plan collision-free to a pre-start pose 7 cm behind the door's outer face
door=rb._box("door",(-0.345,-0.49,1.005),(0.25,0.03,0.21),yaw=float(np.arctan2(-0.24,-0.07)))
rb.apply_scene(R,[door]); print("door box added")
p_pre,qt=pose(-112,0.08)
qpre=R.ik(p_pre,qt,seed=q0,attempts=5); print("pre dq from q0",round(np.abs(np.array(qpre)-np.array(q0)).max(),3))
v=rb.state_valid(R,qpre); print("pre valid",v)
assert v[0]
plan=rb.plan_to_joints(R,qpre,secs=15.0,attempts=8); assert plan, "plan failed"
print("plan pts",len(plan)); rb.exec_plan(R,plan,tscale=3)
print("at pre: tcp",np.round(R.tcp()[0],4),"jerr",round(np.abs(np.array(R.arm_q())-np.array(qpre)).max(),4))
# 2. remove door box, approach the door face and push along the hinge arc
from moveit_msgs.msg import CollisionObject
co=CollisionObject(); co.id="door"; co.header.frame_id="world"; co.operation=CollisionObject.REMOVE; rb.apply_scene(R,[co])
appr=[pose(-112,o) for o in (0.06,0.04,0.02)]
qa=rb.pose_path(R,appr,seed=qpre,validate=True); assert qa, "approach path failed"
R.move_path(qa,0.6); print("at arc start: tcp",np.round(R.tcp()[0],4))
arc=[pose(phi,0.02) for phi in np.arange(-112,4,4.0)]
qs=rb.pose_path(R,arc,seed=qa[-1],validate=True); assert qs, "arc path failed"
R.move_path(qs,0.5)
qn=np.array(R.arm_q()); err=np.abs(qn-np.array(qs[-1])).max(); print("arc done jerr",round(err,4),"tcp",np.round(R.tcp()[0],4))
if err>0.01:
    R.move(qs[-1],2.0,retries=1); print("after converge tcp",np.round(R.tcp()[0],4),"jerr",round(np.abs(np.array(R.arm_q())-np.array(qs[-1])).max(),4))
# 3. retreat -y then up
p_end,qt_end=pose(0,0.02)
ret=[(p_end+np.array([0,-0.08,0]),qt_end),(p_end+np.array([0,-0.12,0.12]),qt_end)]
qr=rb.pose_path(R,ret,validate=True) or rb.pose_path(R,ret,validate=False)
if qr: R.move_path(qr,0.8)
print("retreat tcp",np.round(R.tcp()[0],4))
# 4. verify with birdview + agentview
B=R.cloud("birdview"); np.save("/workspace/bird8_pc.npy",B); R.snap("birdview","/workspace/bird8.png"); R.snap("agentview","/workspace/agent21.png")
m=np.isfinite(B[...,2])&(B[...,2]>1.04)&(B[...,2]<1.12)&(B[...,0]>-0.45)&(B[...,0]<0.10)&(B[...,1]>-0.70)&(B[...,1]<-0.30)
P=B[m]; print("front-region high pts",len(P))
for x0 in np.arange(-0.44,0.10,0.04):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.04)]
    if len(s): print(f"x {x0:+.2f}: y min {s[:,1].min():+.3f} max {s[:,1].max():+.3f} n {len(s)}")
print("DONE")
EOF
nohup python3 -u door_close.py > door_close.log 2>&1 &
echo started

# openrua op 91
sleep 60; cat /workspace/door_close.log

# openrua op 92
sleep 45; cat /workspace/door_close.log; pgrep -f door_close.py >/dev/null && echo RUNNING || echo EXITED

# openrua op 93
sleep 40; cat /workspace/door_close.log; pgrep -f door_close.py >/dev/null && echo RUNNING || echo EXITED

# openrua op 94
sleep 5; pgrep -f door_close.py >/dev/null && (pkill -f door_close.py; echo killed-idle-process) || echo EXITED

# openrua op 95
pgrep -f door_close.py || echo none-running

# openrua op 96
ps -o pid,stat,etime,cmd -p 2995; kill 2995; sleep 2; pgrep -f door_close.py || echo none-running

# openrua op 97
cat > /workspace/retreat.py <<'EOF'
import rb, numpy as np
R=rb.Robot()
print("tcp",np.round(R.tcp()[0],4))
rb.apply_scene(R,[rb._box("door",(-0.175,-0.375,1.005),(0.25,0.03,0.21))]); print("closed-door box added")
hinge=np.array([-0.30,-0.36]); r=0.16; z=1.02; th=np.deg2rad(45)
n=np.array([0,1.0,0]); rad=np.array([1.0,0,0]); qt=rb.frame_quat(np.sin(th)*n-np.cos(th)*np.array([0,0,1.0]),rad)
p=np.array(R.tcp()[0])
back=[(p+np.array([0,-d,0]),qt) for d in (0.01,0.02,0.03,0.04)]
qb=rb.pose_path(R,back,validate=False); assert qb, "backoff failed"
R.move_path(qb,0.5); print("backed off tcp",np.round(R.tcp()[0],4))
# plan to a high pose above/behind the table edge, pointing down
qgoal=None
for k in range(20):
    try: q=R.ik(np.array([-0.15,-0.25,1.40]),rb.frame_quat([0,0,-1],[0,1,0]),attempts=3)
    except RuntimeError: continue
    if rb.state_valid(R,q)[0]: qgoal=q; break
assert qgoal is not None
plan=rb.plan_to_joints(R,qgoal,secs=15.0,attempts=8); assert plan, "plan failed"
rb.exec_plan(R,plan,tscale=3); print("home tcp",np.round(R.tcp()[0],4))
B=R.cloud("birdview"); np.save("/workspace/bird9_pc.npy",B); R.snap("birdview","/workspace/bird9.png"); R.snap("agentview","/workspace/agent22.png")
m=np.isfinite(B[...,2])&(B[...,2]>0.95)&(B[...,2]<1.12)&(B[...,0]>-0.50)&(B[...,0]<0.10)&(B[...,1]>-0.70)&(B[...,1]<-0.30)
P=B[m]; print("pts",len(P))
for x0 in np.arange(-0.48,0.10,0.04):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.04)]
    if len(s): print(f"x {x0:+.2f}: y min {s[:,1].min():+.3f}  z max {s[:,2].max():.3f} n {len(s)}")
print("DONE")
EOF
nohup python3 -u retreat.py > retreat.log 2>&1 &
sleep 50; cat retreat.log

# openrua op 98
sleep 45; cat /workspace/retreat.log

# openrua op 99
ps aux | grep -E "python3 -u (door_close|retreat)\.py" | grep -v grep || echo "no scripts running"
