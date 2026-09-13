#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
ros2 topic list && echo ---ACTIONS && ros2 action list && echo ---SERVICES && ros2 service list | head -60 && echo ---NODES && ros2 node list

# openrua op 5
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png

# openrua op 6
cat > tf_dump.py <<'EOF'
import rclpy, yaml
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
frames_seen = set()
def cb(m):
    for t in m.transforms: frames_seen.add((t.header.frame_id, t.child_frame_id))
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+4
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)
for p,c in sorted(frames_seen): print(p,"->",c)
print("----")
for f in sorted({c for _,c in frames_seen}):
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(f"{f:40s} xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})")
    except Exception as e:
        print(f"{f:40s} ERR {str(e)[:60]}")
EOF
python3 tf_dump.py

# openrua op 7
cat > pxmap.py <<'EOF'
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; pts=[tuple(map(int,a.split(','))) for a in sys.argv[2:]]
rclpy.init(); node=rclpy.create_node("pxmap"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<2 or not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
d=got["d"]; info=got["i"]
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
np.save(f"{cam}_depth.npy",depth)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
for u,v in pts:
    Z=depth[v,u]; p=R@np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])+T
    print(f"px({u},{v}) depth={Z:.3f} world=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
EOF
python3 pxmap.py birdview 325,295 420,300 250,270 220,215 285,325 215,310 155,190 230,215 400,400 320,240

# openrua op 8
cat > seg.py <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy")
# camera at z=3.0 pointing straight down; world z = 3.0 - depth ; x = (v-cy)*Z/fy - 0.2 ; y=(u-cx)*Z/fx
import cv2
img=cv2.imread("birdview.png")
h,w=d.shape
fx=fy=None
import json
# read camera info intrinsics quickly from earlier: recompute from px mapping: use ratio
# instead just derive: from px(320,240)->(-0.2,0), px(400,400)->(0.38,0.29) at Z=2.1 : fy = 160*2.1/0.58
fy=160*2.1/0.58; fx=80*2.1/0.29
print("fx,fy",fx,fy)
def world(u,v):
    Z=d[v,u]; return ((v-240)*Z/fy-0.2,(u-320)*Z/fx,3.0-Z)
# microwave: things with world z between 1.0 and 1.2 in region u<300
zs=3.0-d
mask=(zs>1.0)&(zs<1.2)
ys,xs=np.where(mask)
print("microwave-height px range u:",xs.min(),xs.max()," v:",ys.min(),ys.max())
# print rows to show shape
for v in range(ys.min(),ys.max()+1,5):
    row=np.where(mask[v])[0]
    if len(row): print(v, row.min(), row.max(), len(row), "world x=%.3f"%world(row.min(),v)[0], "y range %.3f..%.3f"%(world(row.min(),v)[1],world(row.max(),v)[1]))
# mug: z between 0.93 and 1.0, u 300-360
m2=(zs>0.92)&(zs<1.0)
ys,xs=np.where(m2)
for lo,hi in [(290,370),(380,460)]:
    sel=(xs>=lo)&(xs<hi)
    if sel.any():
        uu=xs[sel];vv=ys[sel]
        print("mug px u",uu.min(),uu.max(),"v",vv.min(),vv.max(),"center world",world(int(uu.mean()),int(vv.mean())), "maxz",zs[vv,uu].max())
EOF
python3 seg.py

# openrua op 9
cat > mug.py <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy"); zs=3.0-d
fx=fy=579.31
def world(u,v):
    Z=d[v,u]; return ((v-240)*Z/fy-0.2,(u-320)*Z/fx,3.0-Z)
for name,(u0,u1,v0,v1) in {"yellow":(290,370,270,330),"gray":(390,460,270,330)}.items():
    sub=zs[v0:v1,u0:u1]; m=sub>0.905
    vv,uu=np.where(m); uu+=u0; vv+=v0
    print(name,"px u",uu.min(),uu.max(),"v",vv.min(),vv.max(),"n",len(uu))
    W=np.array([world(u,v) for u,v in zip(uu,vv)])
    print("  x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f"%(W[:,0].min(),W[:,0].max(),W[:,1].min(),W[:,1].max(),W[:,2].min(),W[:,2].max()))
    # rim only (z>0.96)
    r=W[W[:,2]>0.95]
    print("  rim x %.3f..%.3f y %.3f..%.3f center (%.3f,%.3f)"%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max(),r[:,0].mean(),r[:,1].mean()))
    # print z histogram
    print("  z vals",np.round(np.unique(np.round(W[:,2],2)),2))
    # ascii map
    for v in range(vv.min(),vv.max()+1):
        print("  ",v,"".join("#" if zs[v,u]>0.95 else ("+" if zs[v,u]>0.905 else ".") for u in range(uu.min(),uu.max()+1)))
EOF
python3 mug.py

# openrua op 10
cat > mugz.py <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy"); zs=3.0-d
np.set_printoptions(linewidth=250)
for v in range(278,312,2):
    print(v, " ".join(f"{zs[v,u]:.2f}"[1:] if zs[v,u]>0.905 else " .." for u in range(302,348,1)))
EOF
python3 mugz.py

# openrua op 11
python3 pxmap.py sideview 285,240 285,250 265,260 305,260 285,288 285,295 270,300 280,220 350,230 2>&1 | grep px; echo ---agent; python3 pxmap.py agentview 335,220 335,230 300,255 365,255 335,290 335,300 340,250 2>&1 | grep px

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[170:340,130:300]; cv2.imwrite('crop_bird.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); c=im[100:370,0:280]; cv2.imwrite('crop_agent.png',cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[170:300,230:430]; cv2.imwrite('crop_side.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
cat > urdf_dump.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("ud"); got=[]
n.create_subscription(String,"/robot_description",lambda m:got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0]); print(len(got[0]))
EOF
python3 urdf_dump.py && grep -n -A12 '<link name="panda_hand"' robot.urdf | head -40; grep -n -B2 -A10 'name="panda_leftfinger"' robot.urdf | grep -E 'collision|geometry|box|mesh|origin' | head; grep -n 'mesh filename' robot.urdf | head -3

# openrua op 14
python3 - <<'EOF'
import numpy as np, struct, re
def stl_bounds(path):
    data=open(path,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        v=np.array([[float(x) for x in l.split()[1:]] for l in data.decode().splitlines() if l.strip().startswith('vertex')])
    else:
        n=struct.unpack_from('<I',data,80)[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
base="/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/"
for l in ["hand","finger","link7","link6"]:
    lo,hi=stl_bounds(base+l+".stl"); print(l,"min",np.round(lo,3),"max",np.round(hi,3))
EOF
grep -o '<joint name="panda_hand_joint"[^/]*/>.\{0,300\}' robot.urdf | head -3; grep -o '<joint name="panda_finger_joint1".\{0,400\}' robot.urdf | head -2

# openrua op 15
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>&1; ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/ 2>&1; python3 - <<'EOF'
import numpy as np, glob, os
def load(path):
    import xml.etree.ElementTree as ET
    if path.endswith('.dae'):
        t=ET.parse(path); ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        vs=[]
        for fa in t.getroot().iter('{http://www.collada.org/2005/11/COLLADASchema}float_array'):
            a=np.array(fa.text.split(),dtype=float)
            if 'position' in fa.get('id','').lower() or len(a)%3==0: vs.append(a.reshape(-1,3))
        return np.vstack(vs)
    import struct
    data=open(path,'rb').read()
    n=struct.unpack_from('<I',data,80)[0]
    arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return arr['v'].reshape(-1,3)
for p in glob.glob('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/*/hand.*')+glob.glob('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/*/finger.*')+glob.glob('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/*/link7.*'):
    try:
        v=load(p); print(os.path.basename(os.path.dirname(p)),os.path.basename(p),"min",np.round(v.min(0),3),"max",np.round(v.max(0),3))
    except Exception as e: print(p,e)
EOF

# openrua op 16
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper lib for this Panda: IK/FK (world frame, panda_hand tip),
trajectory + gripper actions, joint-state reads. Import or run as CLI:

  python3 rob.py ik  x y z qx qy qz qw      # solve only, print joints
  python3 rob.py go  x y z qx qy qz qw [sec] # IK then move
  python3 rob.py js                          # print joint state
  python3 rob.py fk                          # hand pose (world)
  python3 rob.py grip open|close
  python3 rob.py joints j1,...,j7 [sec]
"""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import TwistStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # panda_link0 in world (from TF)


def quat_from_R(R):
    # returns x,y,z,w
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(zaxis, finger_axis):
    """Hand orientation: hand z (approach) = zaxis, hand y (finger travel)
    = finger_axis (made orthogonal). Returns 3x3."""
    z = np.array(zaxis, float); z /= np.linalg.norm(z)
    y = np.array(finger_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def quat_from_axes(zaxis, finger_axis):
    return quat_from_R(R_from_axes(zaxis, finger_axis))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time() * 1000) % 100000))
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---- kinematics (world frame in/out) ----
    def ik(self, pos_w, quat, seed=None, timeout=30, attempts=1):
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_w, float) - BASE
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        seed = seed if seed is not None else self.arm_q()
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def fk(self, q=None, link="panda_hand"):
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        q = q if q is not None else self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res}")
        ps = res.pose_stamped[0].pose
        pos = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        quat = np.array([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return pos, quat

    # ---- motion ----
    def move_joints(self, q, seconds=3.0, waypoints=None):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def go(self, pos_w, quat, seconds=3.0, seed=None):
        q = self.ik(pos_w, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos_w,3)}", flush=True)
            return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.fk()
        print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)})", flush=True)
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg); self.spin(0.05)


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    if cmd == "js":
        print(r.joints())
    elif cmd == "fk":
        p, q = r.fk(); print("pos", np.round(p, 4), "quat", np.round(q, 4))
    elif cmd in ("ik", "go"):
        v = list(map(float, sys.argv[2:9]))
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        if cmd == "ik":
            q = r.ik(v[:3], v[3:])
            print("IK:", None if q is None else np.round(q, 4))
            if q is not None:
                print("FK check:", np.round(r.fk(q)[0], 4))
        else:
            r.go(v[:3], v[3:], sec)
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "joints":
        q = list(map(float, sys.argv[2].split(",")))
        r.move_joints(q, float(sys.argv[3]) if len(sys.argv) > 3 else 3.0)
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 17
timeout 120 python3 rob.py fk && cat > ikprobe.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
print("current hand fk:", np.round(r.fk()[0],3))
seed=r.arm_q()
cands = {
 "above mug, vertical, fingers x": ((-0.085,0.026,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "above mug, vertical, fingers y": ((-0.012,0.076,1.16), quat_from_axes((0,0,-1),(0,1,0))),
 "front of door vertical (-0.085,-0.39,1.16)": ((-0.085,-0.39,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "front of door vertical (-0.135,-0.42,1.16)": ((-0.135,-0.42,1.16), quat_from_axes((0,0,-1),(1,0,0))),
 "horiz +y at (-0.135,-0.44,0.98)": ((-0.135,-0.44,0.98), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.40,1.0)": ((-0.135,-0.40,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.36,1.0)": ((-0.135,-0.36,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.50,1.0)": ((-0.135,-0.50,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz +y at (-0.135,-0.55,1.0)": ((-0.135,-0.55,1.0), quat_from_axes((0,1,0),(1,0,0))),
 "horiz (+x+y) at (-0.135,-0.50,1.0)": ((-0.135,-0.50,1.0), quat_from_axes((0.5,0.866,0),(0.866,-0.5,0))),
 "pitched 45 +y at (-0.135,-0.50,1.05)": ((-0.135,-0.50,1.05), quat_from_axes((0,0.707,-0.707),(1,0,0))),
 "horiz +y fingers z at (-0.135,-0.44,1.0)": ((-0.135,-0.44,1.0), quat_from_axes((0,1,0),(0,0,1))),
 "door push vertical (-0.30,-0.50,1.0)": ((-0.30,-0.50,1.0), quat_from_axes((0,0,-1),(1,0,0))),
 "door push vertical (-0.15,-0.55,1.0)": ((-0.15,-0.55,1.0), quat_from_axes((0,0,-1),(1,0,0))),
 "door push vertical (-0.02,-0.45,1.0)": ((-0.02,-0.45,1.0), quat_from_axes((0,0,-1),(1,0,0))),
}
for k,(p,q) in cands.items():
    sol=r.ik(p,q,seed=seed)
    if sol is None:
        # retry with a different seed
        sol=r.ik(p,q,seed=[0,-0.5,0,-2.0,0,1.6,0.8])
    print(f"{k:50s} ->", "FAIL" if sol is None else np.round(sol,2))
rclpy.shutdown()
EOF
timeout 300 python3 ikprobe.py

# openrua op 18
grep -o '<link name="world"[^>]*>' robot.urdf; grep -o '<joint name="[^"]*" type="fixed"><parent link="world"/>.\{0,200\}' robot.urdf; grep -o 'name="virtual_joint".\{0,200\}' robot.urdf | head -2

# openrua op 19
grep -o '<joint name="[^"]*" type="fixed">.\{0,160\}' robot.urdf | head; grep -c 'world' robot.urdf

# openrua op 20
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK service already speaks world coords (verified vs TF)/' rob.py && timeout 120 python3 rob.py fk && timeout 300 python3 ikprobe.py

# openrua op 21
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Publish/maintain planning-scene collision objects for this scene.
Geometry measured from birdview depth (world frame, table top z=0.900).

  python3 scene.py setup            # table, microwave body, control panel, door, gray mug, yellow mug
  python3 scene.py door <deg>       # re-pose door at opening angle (0 = closed, 118 = as found)
  python3 scene.py remove <id>      # remove an object
  python3 scene.py attach_mug       # attach yellow mug cylinder to the hand (after grasp)
  python3 scene.py detach_mug x y   # detach and re-place standing at x,y
"""
import sys, math
import rclpy
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

TABLE_Z = 0.900
HINGE = (-0.265, -0.325)  # door hinge (vertical axis), world xy
DOOR_LEN = 0.26
DOOR_T = 0.03
MW_TOP = 1.107


def box(id_, cx, cy, cz, sx, sy, sz, yaw=0.0, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[sx, sy, sz])
    p = Pose()
    p.position.x, p.position.y, p.position.z = cx, cy, cz
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = op
    return co


def cyl(id_, cx, cy, cz, h, r, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = "world"
    co.id = id_
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[h, r])
    p = Pose()
    p.position.x, p.position.y, p.position.z = cx, cy, cz
    p.orientation.w = 1.0
    co.primitives = [sp]
    co.primitive_poses = [p]
    co.operation = op
    return co


def door(deg):
    """Door panel as a box hinged at HINGE. deg=0 closed (lies along +x
    face y=-0.325), positive = swung open toward -x/-y (as found: ~118)."""
    ang = -math.radians(deg)  # closed direction +x, opens by rotating about -z
    dx, dy = math.cos(ang), math.sin(ang)
    cx = HINGE[0] + dx * DOOR_LEN / 2 - dy * (-DOOR_T / 2)
    cy = HINGE[1] + dy * DOOR_LEN / 2 + dx * (-DOOR_T / 2)
    return box("mw_door", cx, cy, (TABLE_Z + MW_TOP) / 2, DOOR_LEN, DOOR_T,
               MW_TOP - TABLE_Z, yaw=ang)


def apply(objs, attached=None):
    rclpy.init()
    n = rclpy.create_node("scene_setup")
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=10):
        raise SystemExit("no apply_planning_scene")
    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects = objs
    if attached:
        ps.robot_state.attached_collision_objects = attached
        ps.robot_state.is_diff = True
    fut = cli.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
    print("applied:", fut.result().success if fut.result() else "timeout")
    rclpy.shutdown()


def main():
    cmd = sys.argv[1]
    if cmd == "setup":
        objs = [
            box("table", -0.015, 0.0, TABLE_Z - 0.05, 0.95, 1.10, 0.10),
            box("mw_body", -0.095, -0.225, (TABLE_Z + MW_TOP) / 2, 0.34, 0.20, MW_TOP - TABLE_Z),
            box("mw_panel", 0.035, -0.335, (TABLE_Z + MW_TOP) / 2, 0.08, 0.02, MW_TOP - TABLE_Z),
            door(118.0),
            cyl("gray_mug", -0.001, 0.343, TABLE_Z + 0.09, 0.18, 0.055),
            cyl("yellow_mug", -0.012, 0.026, TABLE_Z + 0.09, 0.18, 0.055),
        ]
        apply(objs)
    elif cmd == "door":
        apply([door(float(sys.argv[2]))])
    elif cmd == "remove":
        co = CollisionObject(); co.id = sys.argv[2]; co.header.frame_id = "world"
        co.operation = CollisionObject.REMOVE
        apply([co])
    elif cmd == "attach_mug":
        # remove world object, attach to hand as a cylinder hanging below fingertips
        rm = CollisionObject(); rm.id = "yellow_mug"; rm.header.frame_id = "world"
        rm.operation = CollisionObject.REMOVE
        aco = AttachedCollisionObject()
        aco.link_name = "panda_hand"
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        co = CollisionObject(); co.header.frame_id = "panda_hand"; co.id = "yellow_mug"
        # approximate: mug axis along hand z, body centred 0.05 along hand -y? keep generic:
        # a 0.11 radius, 0.18 tall cylinder whose top is at fingertip level
        sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[0.18, 0.055])
        p = Pose(); p.position.z = 0.1034 - 0.02 + 0.09; p.position.y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
        p.orientation.w = 1.0
        co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
        aco.object = co
        apply([rm], attached=[aco])
    elif cmd == "detach_mug":
        aco = AttachedCollisionObject(); aco.link_name = "panda_hand"
        aco.object.id = "yellow_mug"; aco.object.operation = CollisionObject.REMOVE
        x, y = float(sys.argv[2]), float(sys.argv[3])
        apply([cyl("yellow_mug", x, y, TABLE_Z + 0.09, 0.18, 0.055)], attached=[aco])


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 22
timeout 120 python3 scene.py setup && timeout 60 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 8}}" 2>&1 | grep -E "id:|dimensions|position" | head -40

# openrua op 23
sed -i 's/req.ik_request.avoid_collisions = False/req.ik_request.avoid_collisions = avoid/; s/def ik(self, pos_w, quat, seed=None, timeout=30, attempts=1):/def ik(self, pos_w, quat, seed=None, timeout=30, avoid=True):/' rob.py && cat > iktest2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
qd=quat_from_axes((0,0,-1),(1,0,0))
for name,p in [("inside microwave body",(-0.1,-0.22,1.0)),("in table",(-0.1,0.2,0.85)),("above mug (fingertips at rim)",(-0.062,0.026,1.16)),("front of door vertical",(-0.085,-0.39,1.16))]:
    s1=r.ik(p,qd,avoid=True); s2=r.ik(p,qd,avoid=False)
    print(f"{name:35s} avoid=True: {'ok' if s1 else 'FAIL'}   avoid=False: {'ok' if s2 else 'FAIL'}", np.round(s1,2) if s1 else "")
rclpy.shutdown()
EOF
timeout 300 python3 iktest2.py

# openrua op 24
cat >> rob.py <<'EOF'


# ---- MoveIt plan+execute (collision-aware) ----
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions


def _plan_client(self):
    if not hasattr(self, "_mg"):
        self._mg = ActionClient(self.node, MoveGroup, M["planning"]["move_action"])
        if not self._mg.wait_for_server(timeout_sec=10):
            raise RuntimeError("no move_action")
    return self._mg


def plan_to_joints(self, q, vel=0.3, acc=0.3, tries=3, planning_time=5.0):
    """Collision-aware plan + execute to joint target q (manifest order)."""
    cli = _plan_client(self)
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = M["planning"]["group"]
    req.num_planning_attempts = 5
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    c = Constraints()
    for j, v in zip(ARM, q):
        jc = JointConstraint(joint_name=j, position=float(v), tolerance_above=0.005,
                             tolerance_below=0.005, weight=1.0)
        c.joint_constraints.append(jc)
    req.goal_constraints = [c]
    goal.request = req
    goal.planning_options = PlanningOptions(plan_only=False, replan=False)
    for t in range(tries):
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            print("  move_group goal rejected", flush=True); continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        res = rf.result()
        code = res.result.error_code.val if res else None
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  plan+exec try{t} code={code} max_joint_err={err:.4f}", flush=True)
        if code == 1 and err < 0.02:
            return True
    return False


def plan_go(self, pos_w, quat, seed=None, **kw):
    q = self.ik(pos_w, quat, seed=seed)
    if q is None:
        print(f"  IK FAILED (collision-aware) for {np.round(pos_w,3)}", flush=True)
        return None
    ok = plan_to_joints(self, q, **kw)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)}) ok={ok}", flush=True)
    return q if ok else None


Robot.plan_to_joints = plan_to_joints
Robot.plan_go = plan_go
EOF
cat > look_inside.py <<'EOF'
import numpy as np, rclpy, subprocess
from rob import Robot, quat_from_axes
r=Robot()
# pitched 45 deg hand pointing +y/down, fingers along x; camera sits 5cm along hand +x (= up/forward here)
q=quat_from_axes((0,0.707,-0.707),(1,0,0))
res=r.plan_go((-0.135,-0.50,1.05), q, vel=0.5, acc=0.5)
print("result", res)
rclpy.shutdown()
EOF
timeout 900 python3 -u look_inside.py 2>&1 | tail -8

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 rob.py fk && ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | head -12

# openrua op 26
cp birdview.png birdview0.png; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import cv2,numpy as np
a=cv2.imread('birdview0.png');b=cv2.imread('birdview.png')
d=np.abs(a.astype(int)-b.astype(int)).sum(2)>40
ys,xs=np.where(d); print('changed px',d.sum(),'bbox u',xs.min(),xs.max(),'v',ys.min(),ys.max())
c=b[150:350,100:320]; cv2.imwrite('crop_bird2.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 27
cat > validity.py <<'EOF'
import rclpy, numpy as np
from rob import Robot, ARM
from moveit_msgs.srv import GetStateValidity
r=Robot()
q=r.arm_q(); print("q",np.round(q,3))
for l in ["panda_hand","panda_leftfinger","panda_link7","panda_link6"]:
    print(l, np.round(r.fk(q,l)[0],3))
cli=r.node.create_client(GetStateValidity,"/check_state_validity"); cli.wait_for_service(5)
req=GetStateValidity.Request(); req.group_name="panda_arm"
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
fut=cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result(); print("valid",res.valid)
for c in res.contacts: print(" contact",c.contact_body_1,c.contact_body_2,np.round([c.position.x,c.position.y,c.position.z],3), "depth",round(c.depth,4))
rclpy.shutdown()
EOF
timeout 120 python3 validity.py

# openrua op 28
cat > lift.py <<'EOF'
import rclpy, numpy as np
from rob import Robot
r=Robot()
p,q=r.fk(); print("at",np.round(p,3))
target=(p[0],p[1],p[2]+0.10)
sol=r.ik(target,q,avoid=True)
print("ik",None if sol is None else np.round(sol,3))
if sol is not None:
    r.move_joints(sol,2.0)
    print("now",np.round(r.fk()[0],3))
rclpy.shutdown()
EOF
timeout 300 python3 -u lift.py

# openrua op 29
timeout 60 python3 pxmap.py birdview 200,200 >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy"); zs=3.0-d
fx=fy=579.31
def world(u,v):
    Z=d[v,u]; return ((v-240)*Z/fy-0.2,(u-320)*Z/fx,3.0-Z)
mask=(zs>1.0)&(zs<1.25)
ys,xs=np.where(mask)
print("door/mw rows (z>1.0):")
for v in range(150,235,3):
    row=np.where(mask[v]&(np.arange(640)<240))[0]
    if len(row):
        zz=zs[v,row]
        print(v, "u",row.min(),row.max(), "x=%.3f y %.3f..%.3f"%(world(row.min(),v)[0],world(row.min(),v)[1],world(row.max(),v)[1]), "zmax %.3f"%zz.max())
EOF

# openrua op 30
sed -i 's/^DOOR_T = 0.03/DOOR_T = 0.07/; s/^MW_TOP = 1.107/MW_TOP = 1.107\nMARGIN = 0.03/' scene.py && python3 - <<'EOF'
import re
s=open('scene.py').read()
s=s.replace('''    return box("mw_door", cx, cy, (TABLE_Z + MW_TOP) / 2, DOOR_LEN, DOOR_T,
               MW_TOP - TABLE_Z, yaw=ang)''','''    return box("mw_door", cx, cy, (TABLE_Z + MW_TOP + MARGIN) / 2, DOOR_LEN + 0.04, DOOR_T,
               MW_TOP + MARGIN - TABLE_Z, yaw=ang)''')
s=s.replace('box("mw_body", -0.095, -0.225, (TABLE_Z + MW_TOP) / 2, 0.34, 0.20, MW_TOP - TABLE_Z)','box("mw_body", -0.095, -0.225, (TABLE_Z + MW_TOP + MARGIN) / 2, 0.34 + 0.02, 0.20, MW_TOP + MARGIN - TABLE_Z)')
open('scene.py','w').write(s)
EOF
timeout 60 python3 scene.py door 118 && cat > look2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0.707,-0.707),(1,0,0))
for p in [(-0.10,-0.52,1.10),(-0.12,-0.50,1.06)]:
    res=r.plan_go(p, q, vel=0.3, acc=0.3)
    if res is not None: break
rclpy.shutdown()
EOF
timeout 900 python3 -u look2.py 2>&1 | tail -6

# openrua op 31
timeout 60 python3 rob.py fk; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png

# openrua op 32
cat > eih_cloud.py <<'EOF'
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam="robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(Image,f"/{cam}/color/image_raw",lambda m:got.setdefault("c",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<3 or not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
d=got["d"]; info=got["i"]; c=got["c"]
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
col=np.frombuffer(c.data,dtype=np.uint8).reshape(c.height,c.width,-1)[:,:,:3]
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print("cam at",np.round(T,3),"optical z axis",np.round(R[:,2],3))
vs,us=np.mgrid[0:d.height,0:d.width]
Z=depth
P=np.stack([(us-cx)*Z/fx,(vs-cy)*Z/fy,Z],-1)@R.T+T
np.save("eih_world.npy",P); np.save("eih_col.npy",col)
ok=np.isfinite(Z)&(Z>0.05)&(Z<3)
W=P[ok]
print("world bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(W[:,0].min(),W[:,0].max(),W[:,1].min(),W[:,1].max(),W[:,2].min(),W[:,2].max()))
# interior analysis: points inside microwave footprint x in (-0.265,-0.005), y>-0.325
inside=W[(W[:,0]>-0.26)&(W[:,0]<-0.01)&(W[:,1]>-0.33)&(W[:,1]<-0.10)]
print("n inside",len(inside))
# floor: z histogram
h,e=np.histogram(inside[:,2],bins=np.arange(0.85,1.20,0.01))
for hh,ee in zip(h,e): 
    if hh>0: print(f"  z {ee:.2f}: {hh}")
# back wall: y histogram of points with z between 0.95 and 1.05
mid=inside[(inside[:,2]>0.95)&(inside[:,2]<1.05)]
h,e=np.histogram(mid[:,1],bins=np.arange(-0.33,-0.10,0.01))
print("y hist (z .95-1.05):"); 
for hh,ee in zip(h,e):
    if hh>0: print(f"  y {ee:.2f}: {hh}")
h,e=np.histogram(mid[:,0],bins=np.arange(-0.27,0.0,0.01))
print("x hist (z .95-1.05):")
for hh,ee in zip(h,e):
    if hh>0: print(f"  x {ee:.2f}: {hh}")
EOF
timeout 120 python3 eih_cloud.py

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null; timeout 60 python3 pxmap.py frontview 335,300 335,305 335,310 335,360 335,365 335,370 335,375 320,335 360,335 2>&1 | grep px

# openrua op 34
python3 - <<'EOF'
import numpy as np, cv2
# frontview: find the yellow mug column profile: for column u=335, print world z along v
import subprocess
EOF
timeout 60 python3 pxmap.py frontview 338,318 338,322 338,326 338,330 338,340 338,350 338,380 338,385 338,390 338,395 338,400 2>&1 | grep px

# openrua op 35
cat > cloud_any.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m:got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m:got.setdefault("i",m),1)
while len(got)<2 or not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
d=got["d"]; info=got["i"]
print("depth",d.width,d.height,d.encoding,"info",info.width,info.height,"k",np.round(info.k,1))
depth=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
vs,us=np.mgrid[0:d.height,0:d.width]
P=np.stack([(us-cx)*depth/fx,(vs-cy)*depth/fy,depth],-1)@R.T+T
np.save(f"{cam}_world.npy",P)
rclpy.shutdown()
EOF
timeout 60 python3 cloud_any.py frontview && python3 - <<'EOF'
import numpy as np
P=np.load("frontview_world.npy")
# yellow mug region: x in (-0.08,0.06), y in (-0.06,0.09)
m=(P[:,:,0]>-0.08)&(P[:,:,0]<0.06)&(P[:,:,1]>-0.03)&(P[:,:,1]<0.09)&(P[:,:,2]>0.905)
vs,us=np.where(m); print("mug px bbox u",us.min(),us.max(),"v",vs.min(),vs.max())
Z=P[:,:,2][m]; print("z max %.3f"%Z.max(), "z percentiles",np.round(np.percentile(Z,[50,90,99,100]),3))
# print column profile at u center
u=int(us.mean())
for v in range(vs.min()-3,vs.max()+3,3):
    p=P[v,u]; print(v, np.round(p,3))
EOF

# openrua op 36
cat > look_handle.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0.707,-0.707),(1,0,0))
res=r.plan_go((-0.01,-0.30,1.20), q, vel=0.3, acc=0.3)
rclpy.shutdown()
EOF
timeout 900 python3 -u look_handle.py 2>&1 | tail -4; timeout 60 python3 rob.py fk; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 37
cat > look_handle.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0.5,-0.866),(1,0,0))
res=r.plan_go((-0.01,-0.17,1.33), q, vel=0.3, acc=0.3)
rclpy.shutdown()
EOF
timeout 900 python3 -u look_handle.py 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 38
timeout 120 python3 cloud_any.py robot0_eye_in_hand 2>&1 | tail -3
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
# mug body region: cylinder centre (-0.012,0.026)
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.abs(X+0.012)<0.08)&(Y>-0.15)&(Y<0.12)&(Z>0.905)&(Z<1.05)
pts=P[m]
print("mug+handle pts",len(pts))
# radial profile of body: for each z band, max radius from centre (excluding handle y<-0.04)
for z0 in np.arange(0.90,1.02,0.01):
    b=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(b)==0: continue
    body=b[b[:,1]>-0.045]
    r=np.hypot(body[:,0]+0.012,body[:,1]-0.026) if len(body) else np.array([0])
    h=b[b[:,1]<=-0.045]
    print(f"z {z0:.2f}: n={len(b)} body r max {r.max():.3f} | handle n={len(h)} y min {h[:,1].min() if len(h) else np.nan:.3f} y max {h[:,1].max() if len(h) else np.nan:.3f} x {h[:,0].min() if len(h) else np.nan:.3f}..{h[:,0].max() if len(h) else np.nan:.3f}")
EOF

# openrua op 39
grep -n "579\|camera_info\|fx" eih_cloud.py cloud_any.py pxmap.py | head -20

# openrua op 40
python3 - <<'EOF'
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy')
X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.abs(X+0.012)<0.09)&(np.abs(Y-0.026)<0.09)&(Z>0.995)&(Z<1.03)
pts=P[m]
print(len(pts), "z range", pts[:,2].min(), pts[:,2].max())
# algebraic circle fit in xy
x,y=pts[:,0],pts[:,1]
A=np.c_[2*x,2*y,np.ones_like(x)]; b=x**2+y**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print("rim circle centre",c[:2],"r",r)
print("x extent",x.min(),x.max(),"y extent",y.min(),y.max())
# rim points near +x
sel=pts[pts[:,0]>x.max()-0.01]; print("+x rim pts mean",sel.mean(0))
EOF

# openrua op 41
timeout 60 python3 scene.py remove yellow_mug 2>&1 | tail -1
cat > grasp1.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
r.gripper(0.04)
q=quat_from_axes((0,0,-1),(1,0,0))
r.plan_go((0.037,0.030,1.20), q, vel=0.3, acc=0.3)
print("fingers",r.fingers())
rclpy.shutdown()
EOF
timeout 900 python3 -u grasp1.py 2>&1 | tail -4

# openrua op 42
cat > grasp2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0,-1),(1,0,0))
r.go((0.037,0.030,1.083), q, seconds=3.0, seed=r.arm_q())
print("hand", r.fk()[0], "fingers", r.fingers())
rclpy.shutdown()
EOF
timeout 600 python3 -u grasp2.py 2>&1 | tail -3; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 43
cat > grasp3.py <<'EOF'
import rclpy
from rob import Robot
r=Robot()
r.gripper(0.0)
print("fingers", r.fingers())
rclpy.shutdown()
EOF
timeout 300 python3 -u grasp3.py 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 44
cat > lift2.py <<'EOF'
import rclpy, numpy as np
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes((0,0,-1),(1,0,0))
r.go((0.037,0.030,1.25), q, seconds=3.0, seed=r.arm_q())
print("hand", r.fk()[0], "fingers", r.fingers())
rclpy.shutdown()
EOF
timeout 600 python3 -u lift2.py 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 45
grep -n "def \|waypoints" rob.py | head -40; cat validity.py | head -40

# openrua op 46
python3 - <<'EOF'
import re
s=open('scene.py').read()
s=s.replace('''    elif cmd == "door":''','''    elif cmd == "cavity":
        # replace solid body with hollow walls: interior x -0.24..-0.03, y -0.325..-0.16, z 0.94..1.095
        rm = CollisionObject(); rm.id = "mw_body"; rm.header.frame_id = "world"; rm.operation = CollisionObject.REMOVE
        x0, x1, y0, y1 = -0.265, 0.075, -0.325, -0.125
        objs = [rm,
            box("mw_floor", (x0+x1)/2, (y0+y1)/2, (TABLE_Z+0.94)/2, x1-x0, y1-y0, 0.94-TABLE_Z),
            box("mw_top", (x0+x1)/2, (y0+y1)/2, (1.095+MW_TOP+MARGIN)/2, x1-x0, y1-y0, MW_TOP+MARGIN-1.095),
            box("mw_back", (x0+x1)/2, (-0.16+y1)/2, (TABLE_Z+MW_TOP)/2, x1-x0, y1+0.16, MW_TOP-TABLE_Z),
            box("mw_left", (x0-0.24)/2, (y0+y1)/2, (TABLE_Z+MW_TOP)/2, -0.24-x0, y1-y0, MW_TOP-TABLE_Z),
            box("mw_right", (-0.03+x1)/2, (y0+y1)/2, (TABLE_Z+MW_TOP)/2, x1+0.03, y1-y0, MW_TOP-TABLE_Z),
        ]
        apply(objs)
    elif cmd == "door":''')
open('scene.py','w').write(s)
EOF
timeout 60 python3 scene.py cavity 2>&1 | tail -1
cat >> rob.py <<'EOF'


def cart_path(self, wps, seconds_per_m=8.0, min_step_t=0.8, max_jump=0.6, avoid=False):
    """wps: list of (pos_w, quat). IK each with previous seed, then one FJT pass.
    Aborts (returns None) if any IK fails or a joint jumps > max_jump rad."""
    seed = self.arm_q()
    qs, ts, t = [], [], 0.0
    prev_p = self.fk()[0]
    for pos, quat in wps:
        q = self.ik(pos, quat, seed=seed, avoid=avoid)
        if q is None:
            print(f"  cart_path IK FAILED at {np.round(pos,3)}", flush=True); return None
        jump = np.abs(np.array(q) - np.array(seed)).max()
        if jump > max_jump:
            print(f"  cart_path joint jump {jump:.2f} at {np.round(pos,3)}", flush=True); return None
        d = np.linalg.norm(np.array(pos) - prev_p)
        t += max(min_step_t, d * seconds_per_m); ts.append(t); qs.append(q)
        seed, prev_p = q, np.array(pos, float)
    wl = list(zip(qs[:-1], ts[:-1]))
    code, err = self.move_joints(qs[-1], ts[-1], waypoints=wl)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(wps[-1][0],3)})", flush=True)
    return qs


Robot.cart_path = cart_path
EOF
echo ok

# openrua op 47
cat > ins_geom.py <<'EOF'
import numpy as np
from rob import R_from_axes, quat_from_axes
# mug in hand coords (pinch at +x rim point, pads 1.5cm below rim)
RIM_C = np.array([0, -0.051, 0.078]); BASE_C = np.array([0, -0.051, 0.178]); RB = 0.033; RR = 0.051
DOOR_Y = -0.325; FLOOR = 0.94; CEIL = 1.095; TOP = 1.107
def pose(theta_deg, mug_x, final_center_y=None, Lz=0.943, H_y=None):
    th = np.radians(theta_deg)
    z_h = np.array([0, np.sin(th), -np.cos(th)]); R = R_from_axes(z_h, (1, 0, 0)); x_h = R[:, 0]
    # lowest base point L = H + R@BASE_C - RB*x_h
    off_L = R @ BASE_C - RB * x_h
    if H_y is None:
        # housing +y corner (finger-base end, upper face) at door plane
        corner = 0.058 * z_h + 0.03 * x_h
        H_y = DOOR_Y - corner[1]
    H = np.array([mug_x + 0.051, H_y, Lz - off_L[2]])
    L = H + off_L
    info = dict(H=H, L=L, final_center_y=L[1] + RB,
                upper_corner=H + 0.03 * x_h, lower_corner=H + 0.058 * z_h + 0.03 * x_h,
                rim_plus_y=H + R @ RIM_C + RR * x_h, base_plus_y=H + R @ BASE_C + RB * x_h)
    return quat_from_axes(z_h, (1, 0, 0)), info
if __name__ == "__main__":
    for th in (30, 33, 36):
        q, i = pose(th, -0.17)
        print(th, {k: np.round(v, 3) for k, v in i.items()})
EOF
python3 ins_geom.py

# openrua op 48
cat > ikcheck.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
from ins_geom import pose
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
q33,info=pose(33,-0.17)
H=info['H']; H0=H+np.array([0,-0.10,0])
tests=[("lift",(0.037,0.03,1.36),qv),("over",(-0.119,-0.48,1.36),qv),("tilted high",(-0.119,-0.48,1.36),q33),
       ("H0",H0,q33),("H",H,q33)]
for th,y in ((45,-0.426),(60,-0.441),(60,-0.391),(45,-0.386)):
    z_h=np.array([0,np.sin(np.radians(th)),-np.cos(np.radians(th))])
    Hd=np.array([-0.17,y,1.0])-0.093*z_h
    tests.append((f"drag{th} P_y={y}",Hd,quat_from_axes(z_h,(1,0,0))))
seed=r.arm_q()
for name,p,q in tests:
    s=r.ik(p,q,seed=seed,avoid=True)
    s2=r.ik(p,q,seed=seed,avoid=False) if s is None else s
    print(f"{name:20s} {np.round(p,3)} avoid={'OK' if s else 'FAIL'} free={'OK' if s2 else 'FAIL'}")
    if s: seed=s
rclpy.shutdown()
EOF
timeout 600 python3 ikcheck.py 2>&1 | grep -v "^\[" 

# openrua op 49
cat > transport.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
from ins_geom import pose
def slerp(q0,q1,t):
    q0,q1=np.array(q0),np.array(q1)
    if np.dot(q0,q1)<0: q1=-q1
    d=np.clip(np.dot(q0,q1),-1,1); th=np.arccos(d)
    if th<1e-6: return q0
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
q33,info=pose(33,-0.17); H=info['H']; H0=H+np.array([0,-0.10,0])
print("H",np.round(H,3),"H0",np.round(H0,3))
p=r.fk()[0]
# 1 lift, 2 over, 3 descend
r.cart_path([((p[0],p[1],1.36),qv), ((-0.05,-0.25,1.36),qv), ((-0.119,-0.48,1.36),qv), ((-0.119,-0.482,1.20),qv)])
print("fingers",r.fingers())
# 4 tilt to 33 deg while descending to H0
wps=[]
for t in (0.34,0.67,1.0):
    pos=np.array([-0.119,-0.482,1.20])*(1-t)+H0*t
    wps.append((pos,slerp(qv,q33,t)))
r.cart_path(wps, min_step_t=1.0)
print("hand",np.round(r.fk()[0],3),"quat",np.round(r.fk()[1],3),"target q33",np.round(q33,3),"fingers",r.fingers())
rclpy.shutdown()
EOF
timeout 900 python3 -u transport.py 2>&1 | grep -v "^\[INFO\]"; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 50
timeout 60 python3 tools/perception/cam_snap.py sideview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 51
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 52
timeout 120 python3 cloud_any.py robot0_eye_in_hand 2>&1 | tail -1
python3 - <<'EOF'
import numpy as np
from rob import R_from_axes
from ins_geom import RIM_C, BASE_C
P=np.load('robot0_eye_in_hand_world.npy')
H=np.array([-0.119,-0.482,1.11]); th=np.radians(33)
z_h=np.array([0,np.sin(th),-np.cos(th)]); R=R_from_axes(z_h,(1,0,0))
rim=H+R@RIM_C; base=H+R@BASE_C
print("expected rim centre",np.round(rim,3),"base centre",np.round(base,3))
# points within 0.12 of rim centre and not on floor/table
d=np.linalg.norm(P-rim,axis=-1); m=(d<0.09)&(P[...,2]>0.93)
pts=P[m]; print("mug pts",len(pts))
# project onto mug axis
ax=z_h; rel=pts-rim; s=rel@ax; rad=np.linalg.norm(rel-np.outer(s,ax),axis=1)
for s0 in np.arange(-0.02,0.11,0.02):
    b=(s>=s0)&(s<s0+0.02)
    if b.sum(): print(f"axial {s0:+.2f}..: n={b.sum()} radius max {rad[b].max():.3f} mean {rad[b].mean():.3f}")
print("lowest mug point", pts[pts[:,2].argmin()].round(3))
EOF

# openrua op 53
cat > insert.py <<'EOF'
import numpy as np, rclpy
from rob import Robot
from ins_geom import pose
r=Robot()
q33,info=pose(33,-0.17); H=info['H']
r.cart_path([(H+np.array([0,-0.05,0]),q33),(H,q33)], min_step_t=1.5)
print("hand",np.round(r.fk()[0],3),"fingers",r.fingers())
rclpy.shutdown()
EOF
timeout 600 python3 -u insert.py 2>&1 | grep -v "^\[INFO\]"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 54
timeout 30 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>/dev/null | grep -A3 "force:"

# openrua op 55
cat > release.py <<'EOF'
import numpy as np, rclpy
from rob import Robot
from ins_geom import pose
r=Robot()
q33,info=pose(33,-0.17)
r.gripper(0.04)
p=r.fk()[0]
r.cart_path([(p+np.array([0,-0.06,0.0]),q33),(p+np.array([0,-0.12,0.02]),q33)], min_step_t=1.5)
print("hand",np.round(r.fk()[0],3),"fingers",r.fingers())
rclpy.shutdown()
EOF
timeout 600 python3 -u release.py 2>&1 | grep -v "^\[INFO\]"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 56
timeout 120 python3 cloud_any.py birdview 2>&1 | tail -1
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.30)&(X<0.0)&(Y>-0.62)&(Y<-0.33)&(Z>0.905)&(Z<1.02)
pts=P[m]; print("pts",len(pts))
if len(pts):
    print("x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"z",pts[:,2].min().round(3),pts[:,2].max().round(3))
    # z histogram
    h,e=np.histogram(pts[:,2],bins=np.arange(0.90,1.03,0.01)); print(list(zip(e[:-1].round(2),h)))
    top=pts[pts[:,2]>pts[:,2].max()-0.015]; print("top pts centre",top.mean(0).round(3), "x range",top[:,0].min().round(3),top[:,0].max().round(3),"y range",top[:,1].min().round(3),top[:,1].max().round(3))
EOF

# openrua op 57
cat > clear.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
r.plan_go((-0.05,-0.62,1.40), qv, vel=0.3, acc=0.3)
rclpy.shutdown()
EOF
timeout 900 python3 -u clear.py 2>&1 | grep -v "^\[INFO\]" | tail -2
for c in birdview agentview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 58
cat > clear.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
for tgt in [(-0.10,-0.55,1.40),(-0.15,-0.50,1.45),(0.0,-0.50,1.40)]:
    if r.plan_go(tgt, qv, vel=0.3, acc=0.3): break
rclpy.shutdown()
EOF
timeout 900 python3 -u clear.py 2>&1 | grep -v "^\[INFO\]" | tail -3
for c in birdview agentview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 59
cat > clear.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
qv=quat_from_axes((0,0,-1),(1,0,0))
for tgt in [(0.05,-0.10,1.35),(0.0,0.0,1.35)]:
    if r.plan_go(tgt, qv, vel=0.3, acc=0.3): break
rclpy.shutdown()
EOF
timeout 900 python3 -u clear.py 2>&1 | grep -v "^\[INFO\]" | tail -2
for c in birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 60
timeout 120 python3 cloud_any.py birdview 2>&1 | tail -0
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.30)&(X<0.0)&(Y>-0.62)&(Y<-0.33)&(Z>0.905)&(Z<1.05)
pts=P[m]; print("pts",len(pts))
print("x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"z",pts[:,2].min().round(3),pts[:,2].max().round(3))
# profile along y: for y bands, x range and z max
for y0 in np.arange(-0.50,-0.33,0.01):
    b=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.01)]
    if len(b): print(f"y {y0:.2f}: n={len(b):3d} x {b[:,0].min():.3f}..{b[:,0].max():.3f} zmax {b[:,2].max():.3f}")
# ridge: highest points
top=pts[pts[:,2]>0.985]; print("ridge pts",len(top)); 
if len(top): print("ridge x mean",top[:,0].mean().round(3),"y range",top[:,1].min().round(3),top[:,1].max().round(3))
EOF

# openrua op 61
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
# hand at (0.049,-0.108,1.351) vertical, fingers along x. housing z ~1.29..1.35
m=(np.abs(X-0.049)<0.2)&(np.abs(Y+0.108)<0.12)&(Z>1.27)&(Z<1.36)
pts=P[m]; print("pts",len(pts))
for z0 in np.arange(1.27,1.36,0.01):
    b=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(b): print(f"z {z0:.2f}: n={len(b)} x {b[:,0].min():.3f}..{b[:,0].max():.3f} (w {b[:,0].max()-b[:,0].min():.3f}) y {b[:,1].min():.3f}..{b[:,1].max():.3f} (w {b[:,1].max()-b[:,1].min():.3f})")
# fingers below: z 1.20..1.27
m=(np.abs(X-0.049)<0.2)&(np.abs(Y+0.108)<0.12)&(Z>1.19)&(Z<1.27)
b=P[m]; print("fingers region n",len(b), "x",b[:,0].min().round(3),b[:,0].max().round(3),"y",b[:,1].min().round(3),b[:,1].max().round(3))
EOF

# openrua op 62
timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; timeout 120 python3 cloud_any.py sideview 2>&1 | tail -0
python3 - <<'EOF'
import numpy as np
P=np.load('sideview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.abs(X-0.049)<0.2)&(np.abs(Y+0.108)<0.15)&(Z>1.20)&(Z<1.40)
pts=P[m]; print("pts",len(pts))
for z0 in np.arange(1.20,1.40,0.01):
    b=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(b): print(f"z {z0:.2f}: n={len(b):4d} x {b[:,0].min():.3f}..{b[:,0].max():.3f} (w {b[:,0].max()-b[:,0].min():.3f})  y {b[:,1].min():.3f}..{b[:,1].max():.3f}")
EOF

# openrua op 63
cat > ikcheck2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
def pitched(th, fingers=(1,0,0), sign=1):
    z=np.array([0,sign*np.sin(np.radians(th)),-np.cos(np.radians(th))]); return z, quat_from_axes(z,fingers)
tests=[]
P=np.array([-0.125,-0.49,0.95])
for th in (45,55,35):
    z,q=pitched(th); H=P-0.093*z; tests.append((f"pinch{th}",H,q)); tests.append((f"pre{th}",H-0.08*z,q))
# after lift and rotation: hand pointing -y/down 45, pinch at P+(0,0,0.15)
z,q=pitched(45,sign=-1); Pl=P+np.array([0,0,0.15]); tests.append(("rot45 lifted",Pl-0.093*z,q))
# yawed 90: hand pointing +x/down 45, fingers along y, pinch at rim +y point of mug centred (-0.17,-0.45), rim z 1.005 on table, pinch 1cm below
z=np.array([0.707,0,-0.707]); q=quat_from_axes(z,(0,1,0)); Pt=np.array([-0.17,-0.40,0.995]); tests.append(("yawed set",Pt-0.093*z,q)); tests.append(("yawed high",Pt+np.array([0,0,0.12])-0.093*z,q))
# handle pinch 45 deg: bar at (-0.17, -0.45-0.084, 0.965)
z,q=pitched(45); Pb=np.array([-0.17,-0.534,0.965]); tests.append(("bar pinch",Pb-0.093*z,q)); tests.append(("bar lifted",Pb+np.array([0,0,0.06])-0.093*z,q)); tests.append(("bar inserted",np.array([-0.17,-0.335,1.02])-0.093*z,q))
seed=r.arm_q()
for name,p,q in tests:
    s=r.ik(p,q,seed=seed,avoid=True); s2=s or r.ik(p,q,seed=seed,avoid=False)
    print(f"{name:14s} {np.round(p,3)} avoid={'OK' if s else 'FAIL'} free={'OK' if s2 else 'FAIL'}")
rclpy.shutdown()
EOF
timeout 600 python3 ikcheck2.py 2>&1 | grep -v "^\[" 

# openrua op 64
cat > reach.py <<'EOF'
import numpy as np, rclpy, itertools
from rob import Robot, quat_from_axes
r=Robot()
cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8]]
def q_pitch(th): z=np.array([0,np.sin(np.radians(th)),-np.cos(np.radians(th))]); return quat_from_axes(z,(1,0,0))
for th in (20,30,45):
    q=q_pitch(th)
    for y in (-0.48,-0.50,-0.52,-0.54,-0.56):
        row=[]
        for z in (0.98,1.02,1.06,1.10):
            ok=False
            for s in seeds:
                if r.ik((-0.14,y,z),q,seed=s,avoid=False,timeout=0.3): ok=True;break
            row.append('O' if ok else '.')
        print(f"pitch{th} y={y} z=.98/1.02/1.06/1.10 : {''.join(row)}")
rclpy.shutdown()
EOF
timeout 900 python3 reach.py 2>&1 | grep -v "^\["

# openrua op 65
sed -i 's/if r.ik((-0.14,y,z),q,seed=s,avoid=False,timeout=0.3): ok=True;break/try:\n                    if r.ik((-0.14,y,z),q,seed=s,avoid=False,timeout=0.5): ok=True;break\n                except RuntimeError: pass/' reach.py && sed -i 's/for th in (20,30,45):/for th in (30,45):/' reach.py && timeout 900 python3 reach.py 2>&1 | grep -v "^\["

# openrua op 66
cat > pushmug.py <<'EOF'
import numpy as np, rclpy, subprocess
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
r.gripper(0.04)
print("fingers", r.fingers())
# pre-pose above, then descend
for t in range(3):
    code=r.plan_go((-0.17,-0.527,1.20),qv,seed=None)
    p,_=r.fk(); print("plan_go code",code,"hand",np.round(p,3))
    if np.linalg.norm(p-np.array([-0.17,-0.527,1.20]))<0.01: break
qs=r.cart_path([((-0.17,-0.527,1.13),qv),((-0.17,-0.527,1.0584),qv)],seconds_per_m=10)
print("wrench before", np.round(r.wrench(),1) if hasattr(r,'wrench') else '')
# push +y in 2.5cm steps, checking
for y in (-0.502,-0.477):
    qs=r.cart_path([((-0.17,y,1.0584),qv)],seconds_per_m=20,min_step_t=1.5)
    print("fingers", r.fingers())
# lift
r.cart_path([((-0.17,-0.477,1.20),qv)],seconds_per_m=8)
p,_=r.fk(); print("final hand",np.round(p,3))
rclpy.shutdown()
EOF
timeout 900 python3 -u pushmug.py 2>&1 | grep -v "^\[" | tail -30

# openrua op 67
timeout 300 python3 cloud_any.py birdview 2>&1 | grep -v "^\[" | tail -2; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.915)
Q=P[m]; print("n",len(Q)); print("y range",Q[:,1].min().round(3),Q[:,1].max().round(3),"z max",Q[:,2].max().round(3))
for y in np.arange(-0.52,-0.32,0.01):
    s=Q[(Q[:,1]>y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y {y:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
EOF

# openrua op 68
cat > park.py <<'EOF'
import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
tgt=tuple(float(v) for v in sys.argv[1:4]) if len(sys.argv)>3 else (0.05,-0.10,1.35)
for t in range(3):
    r.plan_go(tgt,qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array(tgt))<0.01: break
print("hand",np.round(p,3)); rclpy.shutdown()
EOF
timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; timeout 300 python3 cloud_any.py birdview 2>&1 | grep -v "^\[" | tail -1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.915)&(P[:,2]<1.05)
Q=P[m]; print("n",len(Q)); print("y range",Q[:,1].min().round(3),Q[:,1].max().round(3),"z max",Q[:,2].max().round(3))
for y in np.arange(-0.52,-0.32,0.01):
    s=Q[(Q[:,1]>y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y {y:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
EOF

# openrua op 69
cat > pushmug2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
X=-0.163; Z=1.048
for t in range(3):
    r.plan_go((X,-0.525,1.20),qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array([X,-0.525,1.20]))<0.01: break
r.cart_path([((X,-0.525,1.12),qv),((X,-0.525,Z),qv)],seconds_per_m=10)
y=-0.525
while y>-0.455:
    y+=0.012
    r.cart_path([((X,y,Z),qv)],seconds_per_m=30,min_step_t=1.2)
    p,_=r.fk(); lag=y-p[1]; print(f"target y {y:.3f} actual {p[1]:.3f} z {p[2]:.3f} lag {lag:.3f}")
    if lag>0.012 or p[2]>Z+0.012:
        print("stalled"); break
r.cart_path([((p[0],p[1],1.20),qv)],seconds_per_m=8)
rclpy.shutdown()
EOF
timeout 900 python3 -u pushmug2.py 2>&1 | grep -E "target|stalled|hand now" | tail -12

# openrua op 70
timeout 900 python3 -u pushmug2.py 2>&1 | grep -v "^\[" | head -30

# openrua op 71
timeout 900 python3 -u pushmug2.py 2>&1 | grep -v "^\[" | sed -n '13,40p'

# openrua op 72
timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; timeout 300 python3 cloud_any.py birdview 2>&1 | grep -v "^\[" | tail -0; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.30)&(P[:,0]<-0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.915)&(P[:,2]<1.05)
Q=P[m]; print("n",len(Q)); print("y range",Q[:,1].min().round(3),Q[:,1].max().round(3),"z max",Q[:,2].max().round(3))
for y in np.arange(-0.52,-0.36,0.01):
    s=Q[(Q[:,1]>y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y {y:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
EOF

# openrua op 73
ls tools/ tools/* | head -30; grep -l "png\|imwrite" *.py | head

# openrua op 74
timeout 120 python3 tools/perception/cam_snap.py birdview 2>&1 | tail -2; timeout 120 python3 tools/perception/cam_snap.py agentview 2>&1 | tail -2; ls -t *.png | head -4

# openrua op 75
timeout 120 python3 tools/perception/cam_snap.py frontview 2>&1 | tail -1; timeout 120 python3 tools/perception/cam_snap.py sideview 2>&1 | tail -1

# openrua op 76
cat > park2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); qv=quat_from_axes((0,0,-1),(1,0,0))
p,_=r.fk(); r.cart_path([((p[0],p[1],1.40),qv)],seconds_per_m=8)
tgt=(0.05,-0.10,1.35)
for t in range(3):
    r.plan_go(tgt,qv,seed=None); p,_=r.fk()
    if np.linalg.norm(p-np.array(tgt))<0.01: break
print("hand",np.round(p,3)); rclpy.shutdown()
EOF
timeout 600 python3 park2.py 2>&1 | grep -v "^\[" | tail -1; timeout 120 python3 tools/perception/cam_snap.py birdview 2>&1 | tail -0; timeout 300 python3 cloud_any.py birdview 2>&1 | tail -0

# openrua op 77
python3 - <<'EOF'
import numpy as np
from PIL import Image
im=Image.open('birdview.png'); im.crop((140,150,340,350)).resize((600,600),Image.NEAREST).save('bird_crop.png')
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.32)&(P[:,0]<-0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.915)&(P[:,2]<1.05)
Q=P[m]; print("n",len(Q)); print("y range",Q[:,1].min().round(3),Q[:,1].max().round(3),"z max",Q[:,2].max().round(3))
for y in np.arange(-0.52,-0.35,0.01):
    s=Q[(Q[:,1]>y)&(Q[:,1]<y+0.01)]
    if len(s): print(f"y {y:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
print("by x:")
for x in np.arange(-0.30,-0.05,0.01):
    s=Q[(Q[:,0]>x)&(Q[:,0]<x+0.01)]
    if len(s): print(f"x {x:.2f}: y {s[:,1].min():.3f}..{s[:,1].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
EOF

# openrua op 78
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.32)&(P[:,0]<-0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.93)&(P[:,2]<1.05)
Q=P[m]; c=Q.mean(0); u,s,vt=np.linalg.svd(Q-c); ax=vt[0]; 
if ax[1]>0: ax=-ax   # point toward rim (-y side)
print("centroid",c.round(3),"axis",ax.round(3),"yaw from -y (deg)",np.degrees(np.arctan2(ax[0],-ax[1])).round(1))
t=(Q-c)@ax; print("extent along axis",t.min().round(3),t.max().round(3))
# perpendicular horizontal direction
perp=np.array([-ax[1],ax[0],0]); perp/=np.linalg.norm(perp)
for tt in np.arange(t.min(),t.max(),0.01):
    s=Q[(t>tt)&(t<tt+0.01)]; w=(s-c)@perp
    print(f"t {tt:.2f}: perp {w.min():.3f}..{w.max():.3f} zmax {s[:,2].max():.3f} n{len(s)} centre_xy {(c+ax*(tt+0.005))[:2].round(3)}")
EOF

# openrua op 79
python3 -c "
from PIL import Image
im=Image.open('birdview.png'); im.crop((165,235,255,325)).resize((720,720),Image.LANCZOS).save('bird_crop2.png')"

# openrua op 80
python3 - <<'EOF'
import numpy as np
W=np.load('birdview_world.npy'); print(W.shape)
for (u,v,name) in [(211,245,'img rim end (upper-right)'),(184,251,'img base end (lower-left)'),(200,256,'img centroid'),(420,295,'gray mug'),(250,222,'mw top-left corner'),(390,60,'robot base')]:
    print(name, (u,v), np.round(W[v,u],3))
EOF

# openrua op 81
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.32)&(P[:,0]<-0.02)&(P[:,1]<-0.33)&(P[:,1]>-0.60)&(P[:,2]>0.906)&(P[:,2]<1.05)
Q=P[m]; c=np.array([-0.152,-0.419,0.98]); ax=np.array([0.678,-0.735,0]); perp=np.array([0.735,0.678,0])
t=(Q-c)@ax; w=(Q-c)@perp
for tt in np.arange(-0.10,0.08,0.01):
    s=(t>tt)&(t<tt+0.01)
    if s.sum(): print(f"t {tt:+.2f}: perp {w[s].min():+.3f}..{w[s].max():+.3f} z {Q[s,2].min():.3f}..{Q[s,2].max():.3f} n{s.sum()}  centre {(c+ax*(tt+0.005))[:2].round(3)}")
print("low points (z<0.93):")
L=Q[Q[:,2]<0.93]; tl=(L-c)@ax; wl=(L-c)@perp
print(" t range",tl.min().round(3),tl.max().round(3)," perp range",wl.min().round(3),wl.max().round(3), "n",len(L))
EOF

# openrua op 82
cat > mugpose.py <<'EOF'
"""Measure lying mug pose from a fresh birdview cloud (arm must be parked). Prints centroid, axis (base->rim), rim/base centres, handle side."""
import numpy as np, subprocess, sys
subprocess.run("timeout 300 python3 cloud_any.py birdview >/dev/null 2>&1", shell=True)
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.34)&(P[:,0]<0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.62)&(P[:,2]>0.93)&(P[:,2]<1.05)
Q=P[m]; c=Q.mean(0); u,s,vt=np.linalg.svd(Q-c); ax=vt[0].copy(); ax[2]=0; ax/=np.linalg.norm(ax)
t=(Q-c)@ax
# rim = wide end: compare widths at both ends
perp=np.array([-ax[1],ax[0],0])
w=(Q-c)@perp
lo=(t<t.min()+0.02); hi=(t>t.max()-0.02)
if np.ptp(w[lo])>np.ptp(w[hi]): ax=-ax; t=-t; perp=-perp; w=-w
c[2]=0.95
rim=c+ax*t.max(); base=c+ax*t.min()
# handle: low points
m2=(P[:,0]>-0.34)&(P[:,0]<0.05)&(P[:,1]<-0.33)&(P[:,1]>-0.62)&(P[:,2]>0.906)&(P[:,2]<0.93)
L=P[m2]; wl=(L-c)@perp; tl=(L-c)@ax; sel=(np.abs(tl)<0.06)&(np.abs(wl)<0.10)
hs=np.sign(np.median(wl[sel])) if sel.sum()>10 else 0
print(f"centroid {c.round(3)} axis(base->rim) {ax.round(3)} yaw_deg {np.degrees(np.arctan2(ax[1],ax[0])).round(1)}")
print(f"rim centre {rim.round(3)} base centre {base.round(3)} len {t.max()-t.min():.3f} handle_side {hs:+.0f} (perp={perp.round(3)}) zmax {Q[:,2].max():.3f}")
np.save('mugpose.npy',np.array([*c,*ax,*rim,*base,hs]))
EOF
python3 mugpose.py

# openrua op 83
sed -i 's/(P\[:,0\]>-0.34)&(P\[:,0\]<0.05)&(P\[:,1\]<-0.33)/(P[:,0]>-0.34)\&(P[:,0]<-0.04)\&(P[:,1]<-0.35)/g' mugpose.py && python3 mugpose.py

# openrua op 84
cat > yawpush.py <<'EOF'
"""Push the lying mug's rim end sideways (toward -perp) to rotate it. args: push_dist_m [t_offset]"""
import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
d=float(sys.argv[1]); toff=float(sys.argv[2]) if len(sys.argv)>2 else 0.04
mp=np.load('mugpose.npy'); c=mp[0:3]; ax=mp[3:6]; rim=mp[6:9]; hs=mp[12]
perp=np.array([-ax[1],ax[0],0])
side=hs if hs!=0 else 1.0          # push from the handle side toward the other side
pdir=-side*perp                    # push direction
s,cth=np.sin(np.radians(30)),np.cos(np.radians(30))
z_h=-pdir*s-np.array([0,0,cth])    # hand leans back away from mug
q=quat_from_axes(z_h,ax)
tip0=c+ax*toff+side*perp*0.062; tip0[2]=0.94
H0=tip0-0.1034*z_h; H1=H0+pdir*d
print("tip0",tip0.round(3),"H0",H0.round(3),"H1",H1.round(3))
r=Robot(); r.gripper(0.0)
above=H0+np.array([0,0,0.15])
for t in range(3):
    r.plan_go(tuple(above),q,seed=None); p,_=r.fk()
    if np.linalg.norm(p-above)<0.01: break
r.cart_path([(tuple(H0+np.array([0,0,0.06])),q),(tuple(H0),q)],seconds_per_m=10)
n=max(2,int(d/0.015))
wps=[(tuple(H0+pdir*d*(i+1)/n),q) for i in range(n)]
r.cart_path(wps,seconds_per_m=30,min_step_t=1.0)
p,_=r.fk(); print("after push hand",p.round(3),"lag",np.round(H1-p,3))
r.cart_path([(tuple(p+np.array([0,0,0.15])),q)],seconds_per_m=8)
rclpy.shutdown()
EOF
timeout 900 python3 -u yawpush.py 0.04 2>&1 | grep -v "^\[" | grep -E "tip0|after|hand now|code" | tail -8

# openrua op 85
sed -i 's/z_h=-pdir\*s-np.array(\[0,0,cth\])    # hand leans back away from mug/z_h=pdir*s-np.array([0,0,cth])     # fingers point down+push dir, housing trails behind (away from mug)/' yawpush.py && grep -n "z_h=" yawpush.py && timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1 && python3 mugpose.py

# openrua op 86
timeout 120 python3 tools/perception/cam_snap.py birdview >/dev/null 2>&1; python3 - <<'EOF'
from PIL import Image
import numpy as np
im=Image.open('birdview.png'); im.crop((150,225,260,335)).resize((660,660),Image.LANCZOS).save('bird_crop2.png')
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.62)&(P[:,2]>0.906)&(P[:,2]<1.05)
Q=P[m]; c=np.array([-0.152,-0.42,0.95]); ax=np.array([-0.772,-0.635,0]); perp=np.array([0.635,-0.772,0])
t=(Q-c)@ax; w=(Q-c)@perp
for tt in np.arange(-0.10,0.10,0.01):
    s=(t>tt)&(t<tt+0.01)
    if s.sum(): print(f"t {tt:+.2f}: perp {w[s].min():+.3f}..{w[s].max():+.3f} z {Q[s,2].min():.3f}..{Q[s,2].max():.3f} n{s.sum()} centre {(c+ax*(tt+0.005))[:2].round(3)}")
EOF

# openrua op 87
ls -la --time-style=full-iso birdview.png birdview_world.npy; date; timeout 120 python3 tools/perception/cam_snap.py birdview 2>&1 | tail -3; ls -la --time-style=full-iso birdview.png

# openrua op 88
python3 -c "
from PIL import Image
im=Image.open('birdview.png'); im.crop((130,200,300,370)).resize((680,680),Image.LANCZOS).save('bird_crop2.png')"

# openrua op 89
cat > mugpose.py <<'EOF'
"""Measure lying mug pose from a fresh birdview cloud (arm must be parked)."""
import numpy as np, subprocess, sys
if '--nosnap' not in sys.argv: subprocess.run("timeout 300 python3 cloud_any.py birdview >/dev/null 2>&1", shell=True)
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.62)
Q=P[box&(P[:,2]>0.93)&(P[:,2]<1.05)]
R=P[box&(P[:,2]>0.98)&(P[:,2]<1.05)]              # top ridge of the lying cone
cr=R.mean(0); u,s,vt=np.linalg.svd(R-cr); ax=vt[0].copy(); ax[2]=0; ax/=np.linalg.norm(ax)
tr=(R-cr)@ax
# rim end = end where ridge is higher
zlo=R[tr<np.percentile(tr,25),2].mean(); zhi=R[tr>np.percentile(tr,75),2].mean()
if zlo>zhi: ax=-ax
perp=np.array([-ax[1],ax[0],0])
c=Q.mean(0); c[2]=0.95
t=(Q-c)@ax; w=(Q-c)@perp
rim=c+ax*t.max(); base=c+ax*t.min()
L=P[box&(P[:,2]>0.906)&(P[:,2]<0.93)]; wl=(L-c)@perp; tl=(L-c)@ax; sel=(np.abs(tl)<0.06)&(np.abs(wl)<0.10)&(np.abs(wl)>0.03)
hs=float(np.sign(np.median(wl[sel]))) if sel.sum()>10 else 0.0
print(f"centroid {c.round(3)} axis(base->rim) {ax.round(3)} yaw_deg {np.degrees(np.arctan2(ax[1],ax[0])).round(1)}  ridge n={len(R)} zlo/zhi {zlo:.3f}/{zhi:.3f}")
print(f"rim centre {rim.round(3)} base centre {base.round(3)} len {t.max()-t.min():.3f} width {w.min():+.3f}..{w.max():+.3f} handle_side {hs:+.0f} (perp={perp.round(3)}) zmax {Q[:,2].max():.3f}")
np.save('mugpose.npy',np.array([*c,*ax,*rim,*base,hs]))
EOF
python3 mugpose.py --nosnap

# openrua op 90
timeout 900 python3 -u yawpush.py 0.05 0.045 2>&1 | grep -v "^\[" | grep -E "tip0|after|hand now|code" | tail -8; timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; python3 mugpose.py

# openrua op 91
timeout 900 python3 -u yawpush.py 0.05 0.045 2>&1 | grep -v "^\[" | tail -12

# openrua op 92
cat > pinchreach.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0]]
def test(p,q):
    for s in seeds:
        try:
            if r.ik(p,q,seed=s,avoid=False,timeout=0.5): return True
        except RuntimeError: pass
    return False
def pinch(c,a,side,th,tin=0.012,zoff=0.022):
    a=np.array(a,float); a/=np.linalg.norm(a); perp=np.array([-a[1],a[0],0]); c=np.array(c,float)
    s,ct=np.sin(np.radians(th)),np.cos(np.radians(th))
    z_h=-a*s-np.array([0,0,ct]); P=c+a*(0.053-tin)+side*perp*0.041+np.array([0,0,zoff]); H=P-0.093*z_h
    return tuple(H),quat_from_axes(z_h,perp)
c=(-0.152,-0.42,0.95)
for name,a in [("current",(0.62,-0.785)),("rot85 (-x,-y)",(-0.7,-0.7)),("rot40 (-y)",(0,-1)),("rim->-x",(-1,0))]:
    for side in (-1,+1):
        row=[]
        for th in (30,45,60):
            H,q=test_args=pinch(c,a,side,th); ok=test(H,q); row.append(f"{th}:{'O' if ok else '.'}({H[0]:+.2f},{H[1]:+.2f})")
        print(f"{name:14s} side{side:+d} "+"  ".join(row))
rclpy.shutdown()
EOF
timeout 900 python3 pinchreach.py 2>&1 | grep -v "^\["

# openrua op 93
sed -i 's/a=np.array(a,float); a\/=np.linalg.norm(a)/a=np.array([a[0],a[1],0.0]); a\/=np.linalg.norm(a)/' pinchreach.py && timeout 900 python3 pinchreach.py 2>&1 | grep -v "^\["

# openrua op 94
cat > yawpush.py <<'EOF'
"""Push the lying mug's rim end sideways (toward -perp) to rotate it CW. args: push_dist_m [t_offset] [pitch_deg] [tip_z]"""
import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
d=float(sys.argv[1]); toff=float(sys.argv[2]) if len(sys.argv)>2 else 0.045
pitch=float(sys.argv[3]) if len(sys.argv)>3 else 20.0; tipz=float(sys.argv[4]) if len(sys.argv)>4 else 0.955
mp=np.load('mugpose.npy'); c=mp[0:3]; ax=mp[3:6]; hs=mp[12]
perp=np.array([-ax[1],ax[0],0]); side=hs if hs!=0 else 1.0
pdir=-side*perp
s,cth=np.sin(np.radians(pitch)),np.cos(np.radians(pitch))
z_h=pdir*s-np.array([0,0,cth])     # fingers point down+push dir, housing trails behind
q=quat_from_axes(z_h,ax); qv=quat_from_axes((0,0,-1),ax)
tip0=c+ax*toff+side*perp*0.062; tip0[2]=tipz
H0=tip0-0.1034*z_h; H1=H0+pdir*d
print("tip0",tip0.round(3),"H0",H0.round(3),"H1",H1.round(3))
r=Robot(); r.gripper(0.0)
above=H0+np.array([0,0,0.14])
ok=False
for t in range(3):
    r.plan_go(tuple(above),q,seed=None); p,_=r.fk()
    if np.linalg.norm(p-above)<0.01: ok=True; break
if not ok:
    # fallback: vertical hand above, then tilt in place
    av=np.array([H0[0],H0[1],1.20])
    for t in range(3):
        r.plan_go(tuple(av),qv,seed=None); p,_=r.fk()
        if np.linalg.norm(p-av)<0.01: break
    r.cart_path([(tuple(above),q)],seconds_per_m=8,min_step_t=2.0)
r.cart_path([(tuple(H0+np.array([0,0,0.06])),q),(tuple(H0),q)],seconds_per_m=10)
n=max(2,int(d/0.015))
wps=[(tuple(H0+pdir*d*(i+1)/n),q) for i in range(n)]
r.cart_path(wps,seconds_per_m=30,min_step_t=1.0)
p,_=r.fk(); print("after push hand",p.round(3),"lag",np.round(H1-p,3))
r.cart_path([(tuple(p+np.array([0,0,0.15])),q)],seconds_per_m=8)
rclpy.shutdown()
EOF
timeout 900 python3 -u yawpush.py 0.05 2>&1 | grep -v "^\[" | tail -12

# openrua op 95
timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; python3 mugpose.py; timeout 900 python3 -u yawpush.py 0.05 0.04 0 0.968 2>&1 | grep -v "^\[" | tail -8

# openrua op 96
timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; python3 mugpose.py

# openrua op 97
cat pinchreach.py | head -60

# openrua op 98
cat > pinchreach2.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0],[0.5,1.2,-0.5,-0.9,0.3,2.0,0.5]]
def test(p,q):
    for s in seeds:
        try:
            sol=r.ik(p,q,seed=s,avoid=False,timeout=0.5)
            if sol: return sol
        except RuntimeError: pass
    return None
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
def pinch(side,th,tin=0.012,zoff=0.022):
    s,ct=np.sin(np.radians(th)),np.cos(np.radians(th))
    z_h=-a*s-np.array([0,0,ct]); P=c+a*(0.053-tin)+side*perp*0.041+np.array([0,0,zoff]); H=P-0.093*z_h
    return tuple(H),quat_from_axes(z_h,perp)
for side in (-1,+1):
    for th in (30,35,40,45,50,60):
        H,q=pinch(side,th); sol=test(H,q)
        print(f"side{side:+d} th{th} H=({H[0]:+.3f},{H[1]:+.3f},{H[2]:.3f}) ", "OK" if sol is not None else "fail", np.round(sol,2) if sol is not None else "")
rclpy.shutdown()
EOF
timeout 600 python3 pinchreach2.py 2>&1 | grep -v "^\["

# openrua op 99
cat > pinchreach3.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0],[0.5,1.2,-0.5,-0.9,0.3,2.0,0.5],[-0.5,1.2,0.5,-0.9,-0.3,2.0,-0.5],[0,1.3,0,-0.8,0,2.1,0.8]]
def test(p,q):
    for s in seeds:
        try:
            sol=r.ik(p,q,seed=s,avoid=False,timeout=0.5)
            if sol: return sol
        except RuntimeError: pass
    return None
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
def pinch(side,th,fsign,tin=0.012,zoff=0.022):
    s,ct=np.sin(np.radians(th)),np.cos(np.radians(th))
    z_h=-a*s-np.array([0,0,ct]); P=c+a*(0.053-tin)+side*perp*0.041+np.array([0,0,zoff]); H=P-0.093*z_h
    return tuple(H),quat_from_axes(z_h,fsign*perp)
for side in (-1,+1):
  for fs in (-1,+1):
    for th in (30,40,45,50,60):
        H,q=pinch(side,th,fs); sol=test(H,q)
        print(f"side{side:+d} f{fs:+d} th{th} H=({H[0]:+.3f},{H[1]:+.3f},{H[2]:.3f}) ", "OK" if sol is not None else "fail", np.round(sol,2) if sol is not None else "")
rclpy.shutdown()
EOF
timeout 900 python3 pinchreach3.py 2>&1 | grep -v "^\["

# openrua op 100
timeout 900 python3 -u yawpush.py 0.07 0.04 0 0.96 2>&1 | grep -v "^\[" | grep -E "after push|move done|hand now" | tail -5; timeout 600 python3 park.py 2>&1 | grep -v "^\[" | tail -1; python3 mugpose.py

# openrua op 101
python3 - <<'EOF'
import numpy as np
from PIL import Image
im=Image.open('birdview.png'); W=np.load('birdview_world.npy')
# find pixel window around mug region
X,Y=W[...,0],W[...,1]
m=(X>-0.36)&(X<0.0)&(Y<-0.30)&(Y>-0.62)
ys,xs=np.where(m); print(ys.min(),ys.max(),xs.min(),xs.max())
im.crop((xs.min(),ys.min(),xs.max(),ys.max())).resize(((xs.max()-xs.min())*3,(ys.max()-ys.min())*3)).save('bird_crop3.png')
Z=W[...,2]; 
for z0 in (0.905,0.93,0.95,0.97,0.99):
    mm=m&(Z>z0)&(Z<1.05); print(z0, mm.sum(), np.round(X[mm].mean(),3) if mm.sum() else None, np.round(Y[mm].mean(),3) if mm.sum() else None)
EOF

# openrua op 102
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.62)
c=np.array([-0.178,-0.437])
for lo,hi in [(0.905,0.93),(0.93,0.96),(0.96,0.99),(0.99,1.05)]:
    S=P[box&(P[:,2]>lo)&(P[:,2]<hi)]
    d=S[:,:2]-c
    print(f"z {lo}-{hi}: n={len(S)} x {S[:,0].min():.3f}..{S[:,0].max():.3f} y {S[:,1].min():.3f}..{S[:,1].max():.3f}")
    # angular histogram
    ang=np.degrees(np.arctan2(d[:,1],d[:,0])); h,_=np.histogram(ang,bins=12,range=(-180,180)); print("   ang hist(-180..180 step30):",h)
EOF

# openrua op 103
python3 - <<'EOF'
import numpy as np
from PIL import Image
W=np.load('birdview_world.npy'); X,Y,Z=W[...,0],W[...,1],W[...,2]
ys,xs=192,145; ye,xe=301,237
z=Z[ys:ye,xs:xe].copy(); z[~np.isfinite(z)]=0.9
img=np.clip((z-0.9)/0.11,0,1); img=(img*255).astype(np.uint8)
Image.fromarray(img).resize(((xe-xs)*4,(ye-ys)*4),Image.NEAREST).save('bird_height3.png')
# print coarse grid of heights (cm above table) every 4 px
for r in range(ys,ye,5):
    print(f"{X[r,xs]:+.2f} "+"".join(f"{int(round((Z[r,c]-0.9)*100)) if np.isfinite(Z[r,c]) else -1:3d}" for c in range(xs,xe,4)))
print("y:   "+"".join(f"{int(round(Y[ys,c]*100)):3d}" for c in range(xs,xe,4)))
EOF

# openrua op 104
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.30)&(P[:,0]<-0.02)&(P[:,1]<-0.36)&(P[:,1]>-0.56)
Q=P[box]
xs=np.arange(-0.30,-0.02,0.01); ys=np.arange(-0.56,-0.36,0.01)
H=np.full((len(xs),len(ys)),-1.0)
ix=((Q[:,0]+0.30)/0.01).astype(int); iy=((Q[:,1]+0.56)/0.01).astype(int)
for i,j,z in zip(ix,iy,Q[:,2]):
    if 0<=i<len(xs) and 0<=j<len(ys): H[i,j]=max(H[i,j],z)
print("      y="+"".join(f"{int(round(y*100)):3d}" for y in ys))
for i,x in enumerate(xs):
    print(f"x={x:+.2f} "+"".join(f"{int(round((h-0.9)*100)) if h>0 else 0:3d}" if h>0 else "  ." for h in H[i]))
EOF

# openrua op 105
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
# handle region
m=(P[:,0]>-0.155)&(P[:,0]<-0.09)&(P[:,1]<-0.37)&(P[:,1]>-0.47)&(P[:,2]>0.905)
S=P[m]; print("handle pts",len(S)); 
for lo in np.arange(0.90,1.02,0.01):
    s=S[(S[:,2]>=lo)&(S[:,2]<lo+0.01)]
    if len(s): print(f"z {lo:.2f}: n={len(s):3d} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}")
# body cross-section at y=-0.48 (rim half) and y=-0.41
for y0 in (-0.49,-0.47,-0.44,-0.41):
    s=P[(P[:,1]>y0-0.005)&(P[:,1]<y0+0.005)&(P[:,0]>-0.30)&(P[:,0]<-0.05)&(P[:,2]>0.905)]
    o=np.argsort(s[:,0]); s=s[o]
    print(f"y={y0}: "+" ".join(f"({x:.3f},{z:.3f})" for x,z in zip(s[::max(1,len(s)//14),0],s[::max(1,len(s)//14),2])))
EOF

# openrua op 106
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.30)&(P[:,0]<-0.155)&(P[:,1]<-0.36)&(P[:,1]>-0.56)&(P[:,2]>0.93)&(P[:,2]<1.05)
Q=P[box]; c=Q.mean(0); u,s,vt=np.linalg.svd(Q[:,:2]-c[:2]); ax=vt[0]; print("pca axis",ax, "sv",s[:2])
# slice along candidate axis: use top height per slice to find rim(high) vs base(low)
for ang in range(-100,-40,5):
    a=np.array([np.cos(np.radians(ang)),np.sin(np.radians(ang))]); perp=np.array([-a[1],a[0]])
    t=(Q[:,:2]-c[:2])@a; w=(Q[:,:2]-c[:2])@perp
    # width of the footprint perpendicular should be minimal (=diameter) for correct axis
    print(ang, f"t {t.min():+.3f}..{t.max():+.3f} w {w.min():+.3f}..{w.max():+.3f}  zmax lo-half {Q[t<0,2].max():.3f} hi-half {Q[t>0,2].max():.3f}")
EOF

# openrua op 107
cat > conefit.py <<'EOF'
import numpy as np, sys
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.60)&(P[:,2]>0.94)&(P[:,2]<1.05)
Q=P[box]
R0,R1,L=0.035,0.051,0.106   # base radius, rim radius, length
def score(cx,cy,yaw):
    a=np.array([np.cos(yaw),np.sin(yaw),0]); perp=np.array([-a[1],a[0],0]); c=np.array([cx,cy,0.95])
    d=Q-c; t=d@a; w=d@perp; h=d[:,2]
    r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    res=np.abs(r-rt); res[np.abs(t)>L/2+0.005]=0.05   # outside ends
    res=np.minimum(res,0.02)  # robust cap (handle points)
    return res.mean(), (np.abs(res)<0.006).sum()
c0=Q.mean(0)
best=None
for yaw in np.radians(np.arange(-180,180,5)):
    for dx in np.arange(-0.04,0.041,0.01):
        for dy in np.arange(-0.04,0.041,0.01):
            s,n=score(c0[0]+dx,c0[1]+dy,yaw)
            if best is None or s<best[0]: best=(s,n,c0[0]+dx,c0[1]+dy,yaw)
s,n,cx,cy,yaw=best
# refine
for it in range(3):
    step=0.005/(it+1); ystep=np.radians(2/(it+1))
    for yy in yaw+np.arange(-3,4)*ystep:
        for dx in np.arange(-2,3)*step:
            for dy in np.arange(-2,3)*step:
                s2,n2=score(cx+dx,cy+dy,yy)
                if s2<s: s,n,bx,by,byaw=s2,n2,cx+dx,cy+dy,yy
    cx,cy,yaw=bx,by,byaw
a=np.array([np.cos(yaw),np.sin(yaw),0]); perp=np.array([-a[1],a[0],0])
print(f"cone fit: c=({cx:.3f},{cy:.3f}) yaw={np.degrees(yaw):.1f} axis(base->rim)={np.round(a[:2],3)} perp={np.round(perp[:2],3)} score={s:.4f} inliers={n}/{len(Q)}")
print(f"rim centre=({cx+a[0]*L/2:.3f},{cy+a[1]*L/2:.3f}) base centre=({cx-a[0]*L/2:.3f},{cy-a[1]*L/2:.3f})")
# handle: points far from cone surface
c=np.array([cx,cy,0.95]); d=Q-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
out=Q[(r-rt>0.012)&(np.abs(t)<L/2+0.01)]
if len(out):
    dd=out-c; print(f"handle pts {len(out)}: t {(dd@a).min():+.3f}..{(dd@a).max():+.3f} perp {(dd@perp).min():+.3f}..{(dd@perp).max():+.3f} z {out[:,2].min():.3f}..{out[:,2].max():.3f}")
    elev=np.degrees(np.arctan2(out[:,2]-0.95,dd@perp)); print("handle elevation deg (median)",np.median(elev), "radius median", np.median(np.hypot(dd@perp,out[:,2]-0.95)))
np.save('mugpose.npy',np.array([cx,cy,0.95,*a,cx+a[0]*L/2,cy+a[1]*L/2,0.95,cx-a[0]*L/2,cy-a[1]*L/2,0.95,0.0]))
EOF
python3 conefit.py

# openrua op 108
cat > handcheck.py <<'EOF'
"""Numerical clearance check of the Franka hand against the lying mug + scene.
Hand frame: y = finger axis, z = pointing (tips at +0.1034), x = thickness.
"""
import numpy as np
R0,R1,L=0.035,0.051,0.106
HINGE=np.array([-0.265,-0.325]); DOOR_DEG=118
def hand_points(gap_half=0.005, fine=0.005):
    pts=[]; lab=[]
    # housing box
    for x in np.arange(-0.03,0.0301,fine):
        for y in np.arange(-0.0995,0.09951,fine):
            for z in np.arange(-0.024,0.0721,fine):
                pts.append((x,y,z)); lab.append('housing')
    # fingers: pad face at |y|=gap_half, body outward 0.012, x +-0.01, z 0.072..0.1034
    for s in (-1,1):
        for x in np.arange(-0.01,0.0101,fine):
            for y in np.arange(gap_half,gap_half+0.0121,fine/2):
                for z in np.arange(0.072,0.10341,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
    return np.array(pts),np.array(lab)
def mug_model(c,a,elev_deg):
    a=np.array([a[0],a[1],0.0]); a/=np.linalg.norm(a); perp=np.array([-a[1],a[0],0]); c=np.array([c[0],c[1],0.95])
    e=np.radians(elev_deg); rho=np.cos(e)*perp+np.sin(e)*np.array([0,0,1.0])
    return c,a,perp,rho
def penetration(P,c,a,perp,rho):
    """returns per-point penetration depth (>0 inside something) and which"""
    d=P-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    dep=np.zeros(len(P)); what=np.array(['']*len(P),dtype=object)
    # body: solid cone (treat as solid incl. cavity for safety) within |t|<L/2
    inb=(np.abs(t)<L/2)&(r<rt); dep[inb]=np.maximum(dep[inb],np.minimum(rt[inb]-r[inb],L/2-np.abs(t[inb]))); what[inb]='body'
    # handle loop: plane through axis along rho; arms at t=-0.033,+0.020 (1cm), radius 0.043..0.085; bar radius 0.075..0.085
    q=d@rho; s=d@np.cross(a,rho)   # s: offset out of handle plane
    inplane=np.abs(s)<0.006
    for tc in (-0.033,0.020):
        arm=inplane&(np.abs(t-tc)<0.006)&(q>0.03)&(q<0.086)
        dep[arm]=np.maximum(dep[arm],0.003); what[arm]='arm'
    bar=inplane&(t>-0.04)&(t<0.027)&(q>0.074)&(q<0.086)
    dep[bar]=np.maximum(dep[bar],0.003); what[bar]='bar'
    # table
    tb=P[:,2]<0.90; dep[tb]=np.maximum(dep[tb],0.90-P[tb,2]); what[tb]='table'
    # microwave body box (incl door frame region) x -0.265..0.075 y -0.325..-0.125 z<1.107 ; panel x -0.005..0.075 y -0.345..-0.325
    mw=(P[:,0]>-0.265)&(P[:,0]<0.075)&(P[:,1]>-0.325)&(P[:,1]<-0.125)&(P[:,2]<1.107)
    dep[mw]=np.maximum(dep[mw],0.005); what[mw]='mw'
    pn=(P[:,0]>-0.005)&(P[:,0]<0.075)&(P[:,1]>-0.345)&(P[:,1]<-0.325)&(P[:,2]<1.107)
    dep[pn]=np.maximum(dep[pn],0.005); what[pn]='panel'
    # door: hinge, angle
    ang=-np.radians(DOOR_DEG); dd=np.array([np.cos(ang),np.sin(ang)]); nn=np.array([-dd[1],dd[0]])
    rel=P[:,:2]-HINGE; along=rel@dd; off=rel@nn
    door=(along>-0.02)&(along<0.28)&(off<0.0)&(off>-0.07)&(P[:,2]<1.107)   # door panel on -n side
    dep[door]=np.maximum(dep[door],0.005); what[door]='door'
    return dep,what
def pose_from(zh,yh):
    zh=np.array(zh,float); zh/=np.linalg.norm(zh); yh=np.array(yh,float); yh-=yh@zh*zh; yh/=np.linalg.norm(yh); xh=np.cross(yh,zh)
    return np.stack([xh,yh,zh],1)  # columns = hand axes in world
def check(H,Rm,gap_half=0.005,mug=None,verbose=True):
    P,lab=hand_points(gap_half); Pw=(Rm@P.T).T+H
    dep,what=penetration(Pw,*mug)
    bad=dep>0
    if verbose:
        if bad.any():
            for w in set(what[bad]):
                for l in set(lab[bad&(what==w)]):
                    m=bad&(what==w)&(lab==l); print(f"   COLLISION {l} vs {w}: n={m.sum()} maxdepth={dep[m].max():.3f}")
        else: print("   clear")
    return dep,what,Pw,lab
def clearance(Pw,lab,c,a,perp):
    """min distance of housing/finger points to the body cone surface and to door/table (approx)"""
    d=Pw-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    inside_t=np.abs(t)<L/2+0.003
    gap=np.where(inside_t,r-rt,np.hypot(np.maximum(0,r-rt),np.abs(t)-L/2))
    out={}
    for l in set(lab): out[l]=gap[lab==l].min()
    out['table']=(Pw[:,2]-0.9).min()
    return out
EOF
echo ok

# openrua op 109
cat > armpinch_check.py <<'EOF'
import numpy as np, rclpy
from handcheck import *
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
ELEV=26.0; ARM_T=0.020; RP=0.059
e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*np.array([0,0,1.0])
mug=(c,a,perp,rho)
r=Robot(); cur=r.arm_q()
seeds=[cur,[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.2,2.3,0.6],[-0.5,0.6,0.3,-1.6,-0.2,2.2,0.3],[0,0.9,0,-1.2,0,2.1,0.8],[0.8,0.7,-0.8,-1.5,0.5,2.0,0.0],[-0.3,1.0,0.5,-1.0,-0.3,1.8,1.0],[0.5,1.2,-0.5,-0.9,0.3,2.0,0.5],[-0.5,1.2,0.5,-0.9,-0.3,2.0,-0.5],[0,1.3,0,-0.8,0,2.1,0.8],[1.0,0.3,-1.0,-2.0,0.3,2.2,-1.0],[-1.0,0.3,1.0,-2.0,-0.3,2.2,1.0]]
def ik(p,q):
    for s in seeds:
        try:
            sol=r.ik(p,q,seed=s,avoid=False,timeout=0.5)
            if sol: return sol
        except RuntimeError: pass
    return None
pad=c+ARM_T*a+RP*rho
print("pad centre",np.round(pad,3))
for ysign in (+1,-1):
    for tilt in (0,10,20):   # tilt of z_h about a toward -perp (away from handle side)
        th=np.radians(tilt); zh=-np.cos(th)*np.array([0,0,1.0])-np.sin(th)*perp*0  # placeholder
        zh=np.array([0,0,-1.0])*np.cos(th)+(-perp)*np.sin(th)
        Rm=pose_from(zh,ysign*a); H=pad-0.093*zh
        print(f"ysign{ysign:+d} tilt{tilt}: H={np.round(H,3)}")
        dep,what,Pw,lab=check(H,Rm,gap_half=0.04,mug=mug)   # open fingers
        cl=clearance(Pw,lab,c,a,perp); print("   clearances:",{k:round(v,3) for k,v in cl.items()})
        q=quat_from_axes(zh,ysign*a); sol=ik(H,q); print("   IK:", "OK "+str(np.round(sol,2)) if sol is not None else "fail")
        # lifted pose
        sol2=ik(H+np.array([0,0,0.20]),q); print("   IK lifted:", "OK" if sol2 is not None else "fail")
# carry poses
for name,H,zh,yh in [("carry",(-0.135,-0.402,1.03),(0,1,0),(0,0,1)),("precarry",(-0.135,-0.55,1.25),(0,1,0),(0,0,1)),("precarry_low",(-0.135,-0.55,1.03),(0,1,0),(0,0,1)),
                     ("rot90",(-0.20,-0.35,1.25),tuple(a),(0,0,1)),("carry_yneg",(-0.135,-0.402,1.03),(0,1,0),(0,0,-1))]:
    q=quat_from_axes(np.array(zh,float),np.array(yh,float)); sol=ik(np.array(H),q); print(f"{name}: IK", "OK "+str(np.round(sol,2)) if sol is not None else "fail")
rclpy.shutdown()
EOF
timeout 900 python3 armpinch_check.py 2>&1 | grep -v "^\["

# openrua op 110
grep -n "def clearance" -A 30 handcheck.py | head -60; grep -n "def check" -A 25 handcheck.py | head -40

# openrua op 111
cat > armpinch_search.py <<'EOF'
import numpy as np
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0])
ELEV=26.0; e=np.radians(ELEV); Z=np.array([0,0,1.0])
rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
mug=(c,a,perp,rho)
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def handle_clear(Pw,lab):
    d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
    dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                            boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                            boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
    return {l:dist[lab==l].min() for l in set(lab)}
res=[]
for tc in (0.026,0.028,0.030,0.032):
  for RP in (0.052,0.054,0.056,0.058,0.060):
    for tilt in (-10,-5,0,5,10,15,20):
        th=np.radians(tilt); zh=-np.cos(th)*Z-np.sin(th)*perp
        Rm=pose_from(zh,a); pad=c+tc*a+RP*rho; H=pad-0.093*zh
        P,lab=hand_points(0.04); Pw=(Rm@P.T).T+H
        dep,what=penetration(Pw,*mug)
        cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
        # exclude finger vs arm pinch target? fingers open: should clear everything
        m=min(min(cl.values()),min(hc.values()))
        res.append((m,tc,RP,tilt,dep.max(),{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3)))
res.sort(key=lambda r:-r[0])
for r in res[:12]: print(r)
EOF
python3 armpinch_search.py

# openrua op 112
cat > armpinch_search2.py <<'EOF'
import numpy as np, sys
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def run(ELEV,ARMS=(0.020,),verbose=False):
    e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z; mug=(c,a,perp,rho)
    def handle_clear(Pw,lab):
        d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
        dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                                boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                                boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
        return {l:dist[lab==l].min() for l in set(lab)}
    res=[]
    for arm in ARMS:
      for dtc in (0.006,0.008,0.010,0.012):
        tc=arm+dtc if arm>0 else arm-dtc
        for RP in (0.050,0.052,0.054,0.056,0.058,0.060):
          for tilt in range(-40,50,5):
            th=np.radians(tilt); zh=-np.cos(th)*Z-np.sin(th)*perp
            Rm=pose_from(zh,a); pad=c+tc*a+RP*rho; H=pad-0.093*zh
            P,lab=hand_points(0.04); Pw=(Rm@P.T).T+H
            dep,what=penetration(Pw,*mug)
            if dep.max()>0: continue
            cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
            m=min(min(cl.values()),min(hc.values()))
            res.append((round(m,4),arm,round(tc,3),RP,tilt,{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3)))
    res.sort(key=lambda r:-r[0])
    return res
for ELEV in (20,26,30,40,50,60,70,80,90):
    res=run(ELEV,ARMS=(0.020,-0.033))
    print(ELEV, res[0] if res else None)
EOF
python3 armpinch_search2.py

# openrua op 113
timeout 60 ros2 param get /robot_state_publisher robot_description 2>/dev/null | grep -n -i -E "finger|hand" | grep -i -E "collision|mesh|box|origin" | head -20

# openrua op 114
python3 - <<'EOF'
import numpy as np, struct
def stl(fn):
    d=open(fn,'rb').read()
    if d[:5]==b'solid':
        v=[list(map(float,l.split()[1:4])) for l in d.decode(errors='ignore').splitlines() if l.strip().startswith('vertex')]
        return np.array(v)
    n=struct.unpack('<I',d[80:84])[0]; a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3)
for f in ['finger','hand']:
    v=stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',v.min(0).round(4),'max',v.max(0).round(4))
v=stl('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/finger.stl')
# finger extents per z slice
for z0 in np.arange(0,0.06,0.005):
    m=(v[:,2]>=z0)&(v[:,2]<z0+0.005)
    if m.any(): print(f"z {z0:.3f}: x {v[m,0].min():+.4f}..{v[m,0].max():+.4f} y {v[m,1].min():+.4f}..{v[m,1].max():+.4f}")
EOF

# openrua op 115
find / -iname "finger*" \( -name "*.stl" -o -name "*.dae" -o -name "*.obj" \) 2>/dev/null | head; find / -iname "hand*.stl" 2>/dev/null | head

# openrua op 116
cat > armpinch_search3.py <<'EOF'
import numpy as np, sys
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
ELEV=float(sys.argv[1]) if len(sys.argv)>1 else 26.0
e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z; mug=(c,a,perp,rho)
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def handle_clear(Pw,lab):
    d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
    dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                            boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                            boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
    return {l:dist[lab==l].min() for l in set(lab)}
res=[]
P,lab=hand_points(0.04)
for tc in (0.026,0.028,0.030):
  for RP in np.arange(0.048,0.072,0.002):
    for soff in np.arange(-0.012,0.0121,0.004):
      for phi in range(-90,91,5):   # z_h = -(cos(phi) s_hat + sin(phi) q_hat)?? param: direction from pad toward tips
        ph=np.radians(phi); zh=-np.cos(ph)*tau-np.sin(ph)*rho   # phi=0: fingers point -tau (from +tau side); phi=90: fingers point -rho (toward axis)
        if zh[2]>-0.1: continue  # keep fingers pointing somewhat downward
        Rm=pose_from(zh,a); pad=c+tc*a+RP*rho+soff*tau; H=pad-0.093*zh
        Pw=(Rm@P.T).T+H
        dep,what=penetration(Pw,*mug)
        if dep.max()>0: continue
        cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
        # the pad must overlap the arm face: pad centre q in 0.048..0.070 approx (pad 2cm long along zh)
        m=min(min(cl.values()),min(hc.values()))
        res.append((round(m,4),tc,round(RP,3),round(soff,3),phi,{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3),np.round(zh,2)))
res.sort(key=lambda r:-r[0])
for r in res[:15]: print(r)
EOF
python3 armpinch_search3.py 26

# openrua op 117
cat > armpinch_search4.py <<'EOF'
import numpy as np, sys
from handcheck import *
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
ELEV=float(sys.argv[1]) if len(sys.argv)>1 else 26.0
e=np.radians(ELEV); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z; mug=(c,a,perp,rho)
def boxdist(x,lo,hi):
    ce=(lo+hi)/2; hf=(hi-lo)/2; d=np.maximum(0,np.abs(x-ce)-hf); return np.linalg.norm(d,axis=1)
def handle_clear(Pw,lab):
    d=Pw-c; t=d@a; q=d@rho; s=d@tau; X=np.stack([t,q,s],1)
    dist=np.minimum.reduce([boxdist(X,np.array([-0.038,0.03,-0.006]),np.array([-0.028,0.086,0.006])),
                            boxdist(X,np.array([0.015,0.03,-0.006]),np.array([0.025,0.086,0.006])),
                            boxdist(X,np.array([-0.04,0.074,-0.006]),np.array([0.027,0.086,0.006]))])
    return {l:dist[lab==l].min() for l in set(lab)}
# pad face sample (hand frame): x +-0.01, z 0.0834..0.1034, y=0
gx,gz=np.meshgrid(np.linspace(-0.009,0.009,7),np.linspace(0.085,0.1024,7)); PAD=np.stack([gx.ravel(),0*gx.ravel(),gz.ravel()],1)
def pad_overlap(H,Rm):
    Pw=(Rm@PAD.T).T+H; d=Pw-c; q=d@rho; s=d@tau
    return ((q>0.043)&(q<0.074)&(np.abs(s)<0.006)).mean()
res=[]
P,lab=hand_points(0.04)
for tc in (0.027,0.029):
  for RP in np.arange(0.048,0.072,0.002):
    for soff in np.arange(-0.016,0.0241,0.002):
      for phi in range(-90,91,5):
        ph=np.radians(phi); zh=-np.cos(ph)*tau-np.sin(ph)*rho
        if zh[2]>-0.1: continue
        Rm=pose_from(zh,a); pad=c+tc*a+RP*rho+soff*tau; H=pad-0.093*zh
        ov=pad_overlap(H,Rm)
        if ov<0.35: continue
        Pw=(Rm@P.T).T+H
        dep,what=penetration(Pw,*mug)
        if dep.max()>0: continue
        cl=clearance(Pw,lab,c,a,perp); hc=handle_clear(Pw,lab)
        m=min(min(cl.values()),min(hc.values()))
        res.append((round(m,4),round(ov,2),tc,round(RP,3),round(soff,3),phi,{k:round(v,3) for k,v in cl.items()},{k:round(v,3) for k,v in hc.items()},np.round(H,3),np.round(zh,2)))
res.sort(key=lambda r:-r[0])
for r in res[:12]: print(r)
EOF
python3 armpinch_search4.py 26

# openrua op 118
for e in 15 40 55 70; do echo "== $e"; python3 armpinch_search4.py $e | head -2; done

# openrua op 119
cat > ikexplore.py <<'EOF'
import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
print("H0",np.round(H0,3),"zh",np.round(zh,3))
r=Robot(); rng=np.random.default_rng(0)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def sols(p,q,n=40):
    out=[]
    for i in range(n):
        s=rng.uniform(lo,hi)
        try: sol=r.ik(p,q,seed=list(s),avoid=False,timeout=0.3)
        except RuntimeError: sol=None
        if sol: out.append(np.round(sol,2))
    return out
poses={"pinch+a":(H0,zh,a),"pinch-a":(H0,zh,-a),"pinchlift+a":(H0+[0,0,0.2],zh,a),"pinchlift-a":(H0+[0,0,0.2],zh,-a),
       "carry+Z":([-0.135,-0.402,1.03],[0,1,0],[0,0,1]),"carry-Z":([-0.135,-0.402,1.03],[0,1,0],[0,0,-1]),
       "pre+Z":([-0.135,-0.50,1.20],[0,1,0],[0,0,1]),"pre-Z":([-0.135,-0.50,1.20],[0,1,0],[0,0,-1]),
       "prelow+Z":([-0.135,-0.50,1.03],[0,1,0],[0,0,1]),"prelow-Z":([-0.135,-0.50,1.03],[0,1,0],[0,0,-1])}
for k,(p,z,y) in poses.items():
    q=quat_from_axes(np.array(z,float),np.array(y,float)); S=sols(np.array(p,float),q)
    print(k,len(S)); 
    for s in S[:6]: print("   ",s)
rclpy.shutdown()
EOF
timeout 1200 python3 ikexplore.py 2>&1 | grep -v "^\["

# openrua op 120
cat > reachgrid.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); rng=np.random.default_rng(1)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def nsol(p,q,n=12):
    k=0;best=None
    for i in range(n):
        try: sol=r.ik(p,q,seed=list(rng.uniform(lo,hi)),avoid=False,timeout=0.25)
        except RuntimeError: sol=None
        if sol: k+=1; best=np.round(sol,2)
    return k,best
for ys in (1,-1):
  q=quat_from_axes(np.array([0,1.0,0]),np.array([0,0,float(ys)]))
  for x in (-0.20,-0.135):
    for y in (-0.55,-0.50,-0.45):
      for z in (1.03,1.15,1.30):
        k,b=nsol(np.array([x,y,z]),q); print(f"yh{ys:+d} x{x} y{y} z{z}: {k}/12 {b}")
rclpy.shutdown()
EOF
timeout 1500 python3 reachgrid.py 2>&1 | grep -v "^\["

# openrua op 121
cat > reachpush.py <<'EOF'
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot(); rng=np.random.default_rng(2)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
def nsol(p,q,n=10):
    k=0;best=None
    for i in range(n):
        try: sol=r.ik(p,q,seed=list(rng.uniform(lo,hi)),avoid=False,timeout=0.25)
        except RuntimeError: sol=None
        if sol: k+=1; best=np.round(sol,2)
    return k,best
for pitch in (0,20,35):
  th=np.radians(pitch); zh=np.array([0,np.cos(th),-np.sin(th)])
  for yh in ((1,0,0),(0,np.sin(th),np.cos(th))):
    q=quat_from_axes(zh,np.array(yh,float))
    for x in (-0.135,-0.17):
      for y in (-0.40,-0.45,-0.50):
        for z in (0.97,1.02):
          k,b=nsol(np.array([x,y,z]),q); print(f"pitch{pitch} yh{np.round(yh,2)} x{x} y{y} z{z}: {k}/10 {b}")
rclpy.shutdown()
EOF
timeout 1500 python3 reachpush.py 2>&1 | grep -v "^\[" | grep -v " 0/10"

# openrua op 122
cat > armpinch.py <<'EOF'
import numpy as np, rclpy, sys, time
from handcheck import *
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
yh=-a; Rm=pose_from(zh,yh); q=quat_from_axes(zh,yh)
print("H0",np.round(H0,3),"zh",np.round(zh,3))
check(H0,Rm,gap_half=0.04,mug=(c,a,perp,rho))
if len(sys.argv)<2 or sys.argv[1]!='go': sys.exit()
r=Robot()
r.gripper(0.04); time.sleep(0.5); print("fingers",r.fingers())
seed=[0.09,1.27,-1.12,-1.48,1.17,1.97,2.29]
above=H0+np.array([0,0,0.15])
rc=r.plan_go(above,q,seed=seed); print("plan_go above:",rc)
if rc!=1:
    rc=r.plan_go(above,q,seed=seed); print("retry:",rc)
    if rc!=1: sys.exit("plan failed")
print("hand at",np.round(r.fk()[0],3))
# descend in steps
wps=[H0+np.array([0,0,z]) for z in (0.10,0.06,0.03,0.015,0.0)]
rc=r.cart_path([(w,q) for w in wps],seconds_per_m=8.0,min_step_t=0.8,max_jump=0.6,avoid=False); print("descend:",rc)
p,_=r.fk(); print("hand at",np.round(p,3),"err",np.round(p-H0,4))
r.gripper(0.0); time.sleep(0.5); f=r.fingers(); print("fingers after close",f)
np.save('pinch_state.npy',np.array([*H0,*q]))
EOF
python3 armpinch.py

# openrua op 123
grep -n "def cart_path\|def plan_go\|def gripper\|def fingers\|def fk\|def go\b\|def move_joints" -A 6 rob.py | head -90

# openrua op 124
sed -n 280,330p rob.py

# openrua op 125
sed -i 's/rc=r.plan_go(above,q,seed=seed); print("plan_go above:",rc)/rc=r.plan_go(above,q,seed=seed); print("plan_go above:",rc is not None)/; s/if rc!=1:\n/XX/' armpinch.py && python3 - <<'EOF'
s=open('armpinch.py').read()
s=s.replace('if rc!=1:\n    rc=r.plan_go(above,q,seed=seed); print("retry:",rc)\n    if rc!=1: sys.exit("plan failed")','if rc is None:\n    rc=r.plan_go(above,q,seed=seed); print("retry:",rc is not None)\n    if rc is None: sys.exit("plan failed")')
s=s.replace('print("descend:",rc)','print("descend:",rc is not None)')
open('armpinch.py','w').write(s)
EOF
grep -n "rc" armpinch.py

# openrua op 126
timeout 900 python3 -u armpinch.py go 2>&1 | grep -v "^\[" | tail -20

# openrua op 127
cat > armpinch2.py <<'EOF'
import numpy as np, rclpy, sys, time
from rob import Robot, quat_from_axes
from handcheck import pose_from
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
yh=-a; q=quat_from_axes(zh,yh)
r=Robot(); p,_=r.fk(); print("start hand",np.round(p,3),"fingers",r.fingers(),"q",np.round(r.arm_q(),2))
wps=[H0+np.array([0,0,z]) for z in (0.10,0.06,0.03,0.015,0.0)]
rc=r.cart_path([(w,q) for w in wps],seconds_per_m=8.0,min_step_t=0.8,max_jump=0.6,avoid=False); print("descend:",rc is not None)
p,_=r.fk(); print("hand at",np.round(p,3),"err",np.round(p-H0,4))
if rc is None: sys.exit()
r.gripper(0.0); time.sleep(0.5); f=r.fingers(); print("fingers after close",f)
np.save('pinch_state.npy',np.array([*H0,*q]))
EOF
timeout 600 python3 -u armpinch2.py 2>&1 | grep -v "^\["

# openrua op 128
timeout 120 python3 -c "
import rclpy
from rob import Robot
r=Robot(); print(r.fingers()); import time; time.sleep(1); print(r.fingers())
" 2>&1 | grep -v "^\["; ls tools/perception/ 2>/dev/null | head; ls *.py | head -50

# openrua op 129
head -30 tools/perception/cam_snap.py; ros2 topic list | grep image_raw

# openrua op 130
timeout 120 python3 tools/perception/cam_snap.py frontview front_pinch.png 2>&1 | grep -v "^\[" ; timeout 120 python3 tools/perception/cam_snap.py sideview side_pinch.png 2>&1 | grep -v "^\["; timeout 120 python3 tools/perception/cam_snap.py birdview bird_pinch.png 2>&1 | grep -v "^\["

# openrua op 131
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pinch.png 2>&1 | grep -v "^\["; python3 -c "
import cv2; im=cv2.imread('bird_pinch.png'); h,w=im.shape[:2]; print(w,h)
crop=im[int(h*0.35):int(h*0.75), int(w*0.05):int(w*0.5)]; cv2.imwrite('bird_pinch_crop.png', cv2.resize(crop,None,fx=2,fy=2))"

# openrua op 132
sed -n 1,40p cloud_any.py

# openrua op 133
cat > fingermeasure.py <<'EOF'
import numpy as np, rclpy, time, subprocess
from rob import Robot, quat_from_axes
r=Robot(); p,qcur=r.fk(); print("hand",np.round(p,3))
st=np.load('pinch_state.npy'); q0=st[3:7]
# lift straight up 0.2 keeping orientation
rc=r.cart_path([(p+[0,0,0.1],q0),(p+[0,0,0.2],q0)],seconds_per_m=6.0,min_step_t=1.0,max_jump=0.8); print("lift",rc is not None)
r.gripper(0.04); time.sleep(0.5); print("fingers",r.fingers())
# pose: hand pointing +x, fingers closing along y, at (-0.15,0.2,1.15)
q=quat_from_axes(np.array([1.0,0,0]),np.array([0,1.0,0]))
H=np.array([-0.15,0.20,1.15])
rc=r.plan_go(H,q,seed=r.arm_q()); print("plan_go",rc is not None)
p,_=r.fk(); print("hand now",np.round(p,3),"q",np.round(r.arm_q(),2))
EOF
timeout 900 python3 -u fingermeasure.py 2>&1 | grep -v "^\["

# openrua op 134
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, rclpy
from rob import Robot
r=Robot(); p,q=r.fk(); print("hand",np.round(p,3),"quat",np.round(q,3))
x,y,z,w=q; R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("x_h",np.round(R[:,0],2),"y_h",np.round(R[:,1],2),"z_h",np.round(R[:,2],2)); print("tips",np.round(p+0.1034*R[:,2],3))
rc=r.cart_path([(p+[0,0,0.08],q),(p+[0,0,0.16],q)],seconds_per_m=6.0,min_step_t=1.0,max_jump=0.8); print("up",rc is not None)
EOF

# openrua op 135
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes(np.array([1.0,0,0]),np.array([0,1.0,0])); H=np.array([-0.15,0.20,1.15])
rc=r.plan_go(H,q,seed=r.arm_q()); print("plan_go",rc is not None)
p,_=r.fk(); print("hand now",np.round(p,3),"q",np.round(r.arm_q(),2))
EOF

# openrua op 136
grep -n "def plan_to_joints" -A 45 rob.py | grep -n -i "scal\|velocity\|allowed\|planner\|num_planning\|tries" 

# openrua op 137
sed -n 245,279p rob.py

# openrua op 138
cat >> rob.py <<'EOF'


def plan_exec(self, q, vel=0.3, acc=0.3, planning_time=5.0, time_scale=2.0, tries=2):
    """Plan only with MoveIt (collision-aware), then execute the returned joint
    trajectory ourselves through FJT with stretched timing (more robust than
    move_group's own execution, which returns -4 partway)."""
    cli = _plan_client(self)
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = M["planning"]["group"]
    req.num_planning_attempts = 5
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = acc
    c = Constraints()
    for j, v in zip(ARM, q):
        c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v),
                                   tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
    req.goal_constraints = [c]
    goal.request = req
    goal.planning_options = PlanningOptions(plan_only=True, replan=False)
    for t in range(tries):
        send = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            print("  move_group goal rejected", flush=True); continue
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result()
        code = res.result.error_code.val if res else None
        if code != 1:
            print(f"  plan try{t} code={code}", flush=True); continue
        jt = res.result.planned_trajectory.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        wps = []
        for pt in jt.points:
            tt = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) * time_scale
            wps.append(([pt.positions[i] for i in idx], tt))
        wps = [w for w in wps if w[1] > 0.05]
        print(f"  planned {len(jt.points)} pts, {wps[-1][1]:.1f}s", flush=True)
        code2, err = self.move_joints(wps[-1][0], wps[-1][1], waypoints=wps[:-1])
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  exec code={code2} max_joint_err={err:.4f}", flush=True)
        if err < 0.02:
            return True
    return False


def plan_go2(self, pos_w, quat, seed=None, **kw):
    q = self.ik(pos_w, quat, seed=seed)
    if q is None:
        print(f"  IK FAILED (collision-aware) for {np.round(pos_w,3)}", flush=True)
        return None
    ok = plan_exec(self, q, **kw)
    p, _ = self.fk()
    print(f"  hand now at {np.round(p,3)} (target {np.round(pos_w,3)}) ok={ok}", flush=True)
    return q if ok else None


Robot.plan_exec = plan_exec
Robot.plan_go2 = plan_go2
EOF
grep -n "def move_joints" -A 24 rob.py | head -30

# openrua op 139
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v "^\["
import numpy as np, rclpy
from rob import Robot, quat_from_axes
r=Robot()
q=quat_from_axes(np.array([1.0,0,0]),np.array([0,1.0,0])); H=np.array([-0.15,0.20,1.15])
rc=r.plan_go2(H,q,seed=r.arm_q()); print("plan_go2",rc is not None)
p,_=r.fk(); print("hand now",np.round(p,3),"q",np.round(r.arm_q(),2))
EOF

# openrua op 140
timeout 120 python3 cloud_any.py birdview 2>&1 | grep -v "^\[" ; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.20)&(P[:,0]<0.0)&(P[:,1]>0.08)&(P[:,1]<0.32)&(P[:,2]>1.08)&(P[:,2]<1.25)
Q=P[m]; print(len(Q))
# x slices: fingers span x from ~-0.078 (H+0.072) to -0.047 (H+0.1034)
for x0 in np.arange(-0.20,0.0,0.01):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.01)]
    if len(s)<3: continue
    ys=np.sort(s[:,1]); 
    # find clusters in y
    gaps=np.where(np.diff(ys)>0.006)[0]
    segs=np.split(ys,gaps+1)
    print(f"x {x0:+.2f}: n={len(s)} zmax={s[:,2].max():.3f} y-segments:",[(round(g.min(),4),round(g.max(),4)) for g in segs if len(g)>2])
EOF

# openrua op 141
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# hand at (-0.15,0.199,1.152) pointing +x ; fingers span x -0.078..-0.047
for x0,x1 in [(-0.095,-0.085),(-0.085,-0.075),(-0.075,-0.065),(-0.065,-0.058),(-0.058,-0.052),(-0.052,-0.046),(-0.046,-0.040)]:
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>0.08)&(P[:,1]<0.32)&(P[:,2]>1.10)
    ys=np.sort(P[m,1])
    if len(ys)==0: print(x0,x1,"none"); continue
    # segments
    segs=[]; s=ys[0]; prev=ys[0]
    for y in ys[1:]:
        if y-prev>0.006: segs.append((round(s,4),round(prev,4))); s=y
        prev=y
    segs.append((round(s,4),round(prev,4)))
    zs=P[m,2]
    print(f"x {x0:.3f}..{x1:.3f} n={len(ys)} zmax={zs.max():.3f} segs={segs}")
EOF

# openrua op 142
python3 conefit.py 2>&1 | tail -15

# openrua op 143
python3 tools/perception/cam_snap.py birdview bv_now.png >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([-0.186,-0.443]); a=np.array([0.423,-0.906]); perp=np.array([0.906,0.423])
d=P[:,:2]-c; t=d@a; s=d@perp; z=P[:,2]-0.95
m=(np.abs(t)<0.09)&(np.abs(s)<0.2)&(P[:,2]>0.905)
# radial distance from axis in perp-z plane
r=np.hypot(s[m],z[m]); tt=t[m]; ss=s[m]; zz=P[m,2]
# points beyond cone body radius+1cm
rc=0.035+(tt+0.053)/0.106*0.016
h=r>rc+0.012
print("handle-like pts",h.sum())
el=np.degrees(np.arctan2(zz[h]-0.95,ss[h]))
import collections
hist=np.histogram(el,bins=np.arange(-180,181,15))
for cnt,b in zip(hist[0],hist[1]): 
    if cnt: print(f"el {b:5.0f}..{b+15:5.0f}: {cnt}")
print("s range of handle pts", ss[h].min(), ss[h].max(), "z", zz[h].min(), zz[h].max())
EOF

# openrua op 144
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([-0.186,-0.443]); a=np.array([0.423,-0.906]); perp=np.array([0.906,0.423])
d=P[:,:2]-c; t=d@a; s=d@perp
m=(np.abs(t)<0.07)&(np.abs(s)<0.11)&(P[:,2]>0.905)&(P[:,2]<1.06)
tt=t[m]; ss=s[m]; zz=P[m,2]
r=np.hypot(ss,zz-0.95)
rc=0.035+(tt+0.053)/0.106*0.016
h=r>rc+0.012
el=np.degrees(np.arctan2(zz[h]-0.95,ss[h]))
hist=np.histogram(el,bins=np.arange(-180,181,15))
for cnt,b in zip(hist[0],hist[1]):
    if cnt: print(f"el {b:5.0f}..{b+15:5.0f}: {cnt}")
print("handle pts",h.sum(),"t range",tt[h].min(),tt[h].max(),"s",ss[h].min(),ss[h].max(),"z",zz[h].min(),zz[h].max())
# door points: z>1.06 region near hinge
m2=(P[:,2]>1.05)&(P[:,0]<-0.2)&(P[:,1]<-0.3)&(P[:,1]>-0.7)
D=P[m2]; print("door-ish pts",len(D))
if len(D):
    # fit line direction from hinge
    dd=D[:,:2]-np.array([-0.265,-0.325]); ang=np.degrees(np.arctan2(dd[:,1],dd[:,0]))
    far=np.linalg.norm(dd,axis=1)
    print("far end", D[far.argmax()], "angles percentiles", np.percentile(ang,[5,50,95]))
EOF

# openrua op 145
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,2]>1.0)&(P[:,1]<-0.36)&(P[:,1]>-0.58)&(P[:,0]<-0.15)&(P[:,0]>-0.4)
D=P[m]; print(len(D), "x range", np.percentile(D[:,0],[2,50,98]), "z", np.percentile(D[:,2],[2,50,98]))
for y0 in np.arange(-0.58,-0.36,0.04):
    mm=(D[:,1]>y0)&(D[:,1]<y0+0.04); 
    if mm.sum(): print(f"y {y0:.2f}: x {D[mm,0].min():.3f}..{D[mm,0].max():.3f} zmax {D[mm,2].max():.3f}")
EOF
python3 scene.py door 90

# openrua op 146
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
c=np.array([-0.186,-0.443]); a=np.array([0.423,-0.906]); perp=np.array([0.906,0.423])
d=P[:,:2]-c; t=d@a; s=d@perp
m=(np.abs(t)<0.07)&(s>0.03)&(s<0.11)&(P[:,2]>0.905)&(P[:,2]<1.06)
tt=t[m]; ss=s[m]; zz=P[m,2]
for t0 in np.arange(-0.06,0.06,0.01):
    mm=(tt>t0)&(tt<t0+0.01)
    if mm.sum()==0: continue
    # points: (s,z); show the outermost s and their z, and max z
    order=np.argsort(ss[mm])
    S=ss[mm][order]; Z=zz[mm][order]
    print(f"t {t0:+.2f}: n={mm.sum():3d} s_max={S[-1]:.3f} z@smax={Z[-1]:.3f}  zmax={Z.max():.3f} s@zmax={S[Z.argmax()]:.3f}  s of pts with z>0.965: {np.round(S[Z>0.965][[0,-1]] if (Z>0.965).sum() else [],3)}")
EOF

# openrua op 147
grep -n "def \|door\|DOOR\|0.012\|0.072" /workspace/handcheck.py | head -60

# openrua op 148
python3 - <<'EOF'
s=open("handcheck.py").read()
s=s.replace("""    # fingers: pad face at |y|=gap_half, body outward 0.012, x +-0.01, z 0.072..0.1034
    for s in (-1,1):
        for x in np.arange(-0.01,0.0101,fine):
            for y in np.arange(gap_half,gap_half+0.0121,fine/2):
                for z in np.arange(0.072,0.10341,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
""","""    # fingers (measured from birdview): pad section z 0.084..0.1034, pad face at |y|=gap_half,
    # body outward 0.014; neck z 0.065..0.084 offset outward gap_half+0.012..+0.026; x +-0.01
    for s in (-1,1):
        for x in np.arange(-0.01,0.0101,fine):
            for y in np.arange(gap_half,gap_half+0.0141,fine/2):
                for z in np.arange(0.084,0.10341,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
            for y in np.arange(gap_half+0.012,gap_half+0.0261,fine/2):
                for z in np.arange(0.065,0.084,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
""")
s=s.replace("""    ang=-np.radians(DOOR_DEG); dd=np.array([np.cos(ang),np.sin(ang)]); nn=np.array([-dd[1],dd[0]])
    rel=P[:,:2]-HINGE; along=rel@dd; off=rel@nn
    door=(along>-0.02)&(along<0.28)&(off<0.0)&(off>-0.07)&(P[:,2]<1.107)   # door panel on -n side
""","""    # door measured at 90 deg open: panel x -0.29..-0.245, y -0.59..-0.325 (handle knob to x -0.31 near y -0.58)
    door=(P[:,0]>-0.295)&(P[:,0]<-0.245)&(P[:,1]>-0.595)&(P[:,1]<-0.325)&(P[:,2]<1.107)
""")
s=s.replace("HINGE=np.array([-0.265,-0.325]); DOOR_DEG=118","HINGE=np.array([-0.265,-0.325]); DOOR_DEG=90")
open("handcheck.py","w").write(s)
EOF
python3 -c "import handcheck as h; P,l=h.hand_points(); print(len(P))"

# openrua op 149
cat > /workspace/barpinch_search.py <<'EOF'
import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho)
B=c+(-0.0075)*a+0.083*rho    # bar centre
print("bar centre",np.round(B,3))
res=[]
for beta in range(0,61,5):
    b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho; zh=-(np.cos(b)*rho-np.sin(b)*tau)
    for sgn in (1,-1):
        Rm=h.pose_from(zh,sgn*yh)
        for delta in (0.004,0.008,0.012):
            for off in (-0.006,0.0,0.006):     # offset along y_h (closing dir) of hand centre from bar
                tip=B+delta*zh+off*yh
                H=tip-0.1034*zh
                worst=0; what=set()
                for g in (0.040,0.005):
                    dep,wh,Pw,lab=h.check(H,Rm,gap_half=g,mug=mug,verbose=False)
                    if dep.max()>worst: worst=dep.max()
                    what|=set(wh[dep>0])
                    cl=h.clearance(Pw,lab,c,a,perp)
                res.append((worst,beta,sgn,delta,off,round(cl['table'],3),round(min(cl[k] for k in cl if k!='table'),3),what,H))
res.sort(key=lambda r:(r[0],-r[5]))
for r in res[:25]: print(r[:8], np.round(r[8],3))
EOF
python3 barpinch_search.py

# openrua op 150
cat > /workspace/barpinch_search2.py <<'EOF'
import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho)
B=c+(-0.0075)*a+0.083*rho
res=[]
for gamma in range(-75,-14,5):
    g=np.radians(gamma)
    for beta in (0,10,20):
        b=np.radians(beta)
        yh=np.cos(b)*tau+np.sin(b)*rho
        zdir=-np.cos(g)*rho+np.sin(g)*a          # in loop plane
        zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
        for sgn in (1,-1):
            Rm=h.pose_from(zh,sgn*yh)
            for delta in (0.006,0.010,0.014):
                tip=B+delta*zh; H=tip-0.1034*zh
                worst=0; what=set(); cl_t=9; cl_b=9
                for gg in (0.040,0.005):
                    dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug,verbose=False)
                    worst=max(worst,dep.max()); what|=set(wh[dep>0])
                    cl=h.clearance(Pw,lab,c,a,perp); cl_t=min(cl_t,cl['table']); cl_b=min(cl_b,min(cl[k] for k in cl if k!='table'))
                wrist=H-0.1*zh
                res.append((round(worst,3),gamma,beta,sgn,delta,round(cl_t,3),round(cl_b,3),what,np.round(H,3),np.round(wrist,3)))
res.sort(key=lambda r:(r[0],-min(r[5],r[6])))
for r in res[:30]: print(r)
EOF
python3 barpinch_search2.py

# openrua op 151
python3 - <<'EOF'
import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
def ev(gamma,beta,delta,sgn=-1,verbose=False):
    g=np.radians(gamma); b=np.radians(beta)
    yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    Rm=h.pose_from(zh,sgn*yh); tip=B+delta*zh; H=tip-0.1034*zh
    out=[]
    for gg in (0.040,0.005):
        dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug,verbose=verbose)
        for l in set(lab):
            m=lab==l; out.append((gg,l,round((Pw[m,2]-0.9).min(),3),round(dep[m].max(),3)))
        cl=h.clearance(Pw,lab,c,a,perp); out.append((gg,'body-clear',{k:round(v,3) for k,v in cl.items()}))
    return H,Rm,out
for beta in (0,-10,-20,-30):
    H,Rm,out=ev(-25,beta,0.006)
    print("beta",beta,np.round(H,3)); 
    for o in out: print("  ",o)
EOF

# openrua op 152
grep -n "def \|^class\|^import\|^from" /workspace/rob.py | head -40; grep -n "def quat_from_axes" -A8 /workspace/rob.py | head -20

# openrua op 153
cat > barpinch_ik.py <<'EOF'
import numpy as np, handcheck as h, rob
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
B=c+(-0.0075)*a+0.083*rho
def pose(gamma,beta,delta,sgn):
    g=np.radians(gamma); b=np.radians(beta)
    yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    Rm=h.pose_from(zh,sgn*yh); tip=B+delta*zh; H=tip-0.1034*zh
    return H,Rm,zh,sgn*yh
r=rob.Robot(); print("q now",np.round(r.arm_q(),2),"fingers",r.fingers())
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(0)
found={}
for gamma in (-20,-25,-30,-35):
  for beta in (-15,-20,-25):
    for sgn in (1,-1):
      H,Rm,zh,yh=pose(gamma,beta,0.008,sgn); quat=rob.quat_from_axes(zh,yh)
      sols=[]
      for k in range(12):
        seed=list(rng.uniform(lo,hi))
        try: q=r.ik(list(H),quat,seed=seed,timeout=1.0,avoid=True)
        except Exception as e: q=None
        if q is not None: sols.append(np.round(q,2))
      print(gamma,beta,sgn,"H",np.round(H,3),"sols",len(sols), sols[0] if sols else "")
      if sols: found[(gamma,beta,sgn)]=(H,quat,sols)
np.save("barpinch_sols.npy",np.array([(k,v[0],v[1],v[2]) for k,v in found.items()],dtype=object),allow_pickle=True)
EOF
timeout 600 python3 barpinch_ik.py 2>&1 | grep -v "^\[" | tail -30

# openrua op 154
python3 - <<'EOF'
import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho; S=np.array([-0.66,0,1.245])
res=[]
for gamma in range(-20,11,5):
  g=np.radians(gamma)
  for beta in range(-50,-19,5):
    b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    for sgn in (1,-1):
      Rm=h.pose_from(zh,sgn*yh)
      for delta in (0.006,0.010):
        tip=B+delta*zh; H=tip-0.1034*zh; wrist=H-0.1*zh
        worst=0; what=set(); cl_t=9; cl_b=9
        for gg in (0.040,0.005):
            dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug,verbose=False)
            m=(dep>0)&(wh!='bar'); worst=max(worst,dep[m].max() if m.any() else 0); what|=set(wh[m])
            cl=h.clearance(Pw,lab,c,a,perp); cl_t=min(cl_t,cl['table']); cl_b=min(cl_b,min(cl[k] for k in cl if k!='table'))
        res.append((round(worst,3),gamma,beta,sgn,delta,round(cl_t,3),round(cl_b,3),what,np.round(H,3),np.round(wrist,3),round(np.linalg.norm(wrist-S),3)))
res.sort(key=lambda r:(r[0],-min(r[5],r[6])))
for r in res[:24]: print(r)
EOF

# openrua op 155
cat > barpinch_ik2.py <<'EOF'
import numpy as np, handcheck as h, rob, pickle
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
def pose(gamma,beta,delta,sgn):
    g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh)
    tip=B+delta*zh; H=tip-0.1034*zh
    return H,zh,sgn*yh
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(1); found={}
for gamma in (-15,-10,-5,0):
  for beta in (-45,-40,-35,-30,-25):
    for sgn in (1,-1):
      H,zh,yh=pose(gamma,beta,0.006,sgn); quat=rob.quat_from_axes(zh,yh)
      sols=[]
      for k in range(10):
        seed=list(rng.uniform(lo,hi))
        try: q=r.ik(list(H),quat,seed=seed,timeout=0.5,avoid=True)
        except Exception: q=None
        if q is not None: sols.append(np.round(q,3))
      print(gamma,beta,sgn,"H",np.round(H,3),"sols",len(sols), sols[0] if sols else "",flush=True)
      if sols: found[(gamma,beta,sgn)]=(H,quat,sols)
pickle.dump(found,open("barpinch_sols.pkl","wb"))
EOF
timeout 900 python3 barpinch_ik2.py 2>&1 | grep -v "^\[" | tail -45

# openrua op 156
python3 - <<'EOF'
import pickle, numpy as np
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
f=pickle.load(open("barpinch_sols.pkl","rb"))
for k,(H,quat,sols) in f.items():
    for q in sols:
        q=np.array(q); marg=np.minimum(q-lo,hi-q).min()
        if q[0]<0 and q[0]>-1.8 and marg>0.15: print(k,np.round(H,3),np.round(q,2),"margin",round(marg,2))
EOF

# openrua op 157
cat > barpinch_go.py <<'EOF'
import numpy as np, handcheck as h, rob, sys, time
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
gamma,beta,delta,sgn=-10,-40,0.006,1
g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh); yh=sgn*yh
tip=B+delta*zh; H=tip-0.1034*zh; Rm=h.pose_from(zh,yh); quat=rob.quat_from_axes(zh,yh)
print("H",np.round(H,3),"zh",np.round(zh,3),"yh",np.round(yh,3))
for gg in (0.040,0.005):
    dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug); print("  gap",gg,{k:round(v,3) for k,v in h.clearance(Pw,lab,c,a,perp).items()})
np.save("barpinch_pose.npy",np.concatenate([H,quat,zh,yh]))
if len(sys.argv)<2: sys.exit()
r=rob.Robot()
q_grasp=np.array([-0.73,1.0,0.22,-1.18,-0.39,1.73,1.54])
q_grasp=np.array(r.ik(list(H),quat,seed=list(q_grasp),timeout=5,avoid=True)); print("q_grasp",np.round(q_grasp,3))
Hpre=H-0.10*zh
q_pre=np.array(r.ik(list(Hpre),quat,seed=list(q_grasp),timeout=5,avoid=True)); print("q_pre",np.round(q_pre,3))
np.save("barpinch_q.npy",np.stack([q_grasp,q_pre]))
if sys.argv[1]=="go":
    r.gripper(0.04)
    ok=r.plan_exec(list(q_pre),vel=0.3,acc=0.3,planning_time=10.0,time_scale=2.0)
    print("plan_exec to pre:",ok,"q now",np.round(r.arm_q(),3))
    p,_=r.fk(); print("hand now",np.round(p,3),"target",np.round(Hpre,3))
EOF
python3 barpinch_go.py check 2>&1 | grep -v "^\["

# openrua op 158
python3 - <<'EOF'
import numpy as np, handcheck as h
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
def boxdist(P,lo,hi):
    d=np.maximum(np.maximum(lo-P,P-hi),0); return np.linalg.norm(d,axis=1).min()
panel=(np.array([-0.005,-0.345,0.9]),np.array([0.075,-0.325,1.107])); mw=(np.array([-0.265,-0.325,0.9]),np.array([0.075,-0.125,1.107]))
door=(np.array([-0.295,-0.595,0.9]),np.array([-0.245,-0.325,1.107]))
for gamma,beta,sgn,delta in [(-10,-40,1,0.006),(-15,-40,1,0.006),(-20,-40,1,0.006),(-10,-45,-1,0.006),(-5,-45,-1,0.006),(-15,-45,1,0.006),(-15,-35,-1,0.006),(-20,-35,-1,0.006)]:
    g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh); yh=sgn*yh
    tip=B+delta*zh; H=tip-0.1034*zh; Rm=h.pose_from(zh,yh)
    dep,wh,Pw,lab=h.check(H,Rm,gap_half=0.04,mug=mug,verbose=False)
    cl=h.clearance(Pw,lab,c,a,perp)
    wrist=H-0.1*zh
    print(gamma,beta,sgn,"panel",round(boxdist(Pw,*panel),3),"mw",round(boxdist(Pw,*mw),3),"door",round(boxdist(Pw,*door),3),"body",round(min(cl['finger+1'],cl['finger-1'],cl['housing']),3),"table",round(cl['table'],3),"wrist",np.round(wrist,3),"wrist-panel",round(boxdist(wrist[None],*panel),3))
EOF

# openrua op 159
cat > barpinch_ik3.py <<'EOF'
import numpy as np, handcheck as h, rob, pickle
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
B=c+(-0.0075)*a+0.083*rho
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
rng=np.random.default_rng(2)
for gamma,beta,sgn in [(-20,-40,1),(-20,-40,-1),(-20,-35,-1),(-20,-35,1),(-25,-40,1),(-25,-40,-1),(-20,-45,1),(-20,-45,-1)]:
    g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
    zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh); yh=sgn*yh
    H=B+0.006*zh-0.1034*zh; quat=rob.quat_from_axes(zh,yh)
    best=None
    for k in range(12):
        seed=list(rng.uniform(lo,hi))
        if k==0: seed=[-0.62,1.08,0.04,-1.01,-0.18,1.59,1.44*sgn]
        try: q=r.ik(list(H),quat,seed=seed,timeout=0.5,avoid=True)
        except Exception: q=None
        if q is not None:
            q=np.array(q); marg=np.minimum(q-lo,hi-q).min()
            if -1.8<q[0]<0 and (best is None or marg>best[1]): best=(np.round(q,3),round(marg,2))
    print(gamma,beta,sgn,np.round(H,3),best,flush=True)
EOF
timeout 600 python3 barpinch_ik3.py 2>&1 | grep -v "^\["

# openrua op 160
sed -i 's/gamma,beta,delta,sgn=-10,-40,0.006,1/gamma,beta,delta,sgn=-20,-40,0.006,1/; s/q_grasp=np.array(\[-0.73,1.0,0.22,-1.18,-0.39,1.73,1.54\])/q_grasp=np.array([-0.567,1.23,-0.11,-0.742,-0.001,1.389,1.376])/' barpinch_go.py && timeout 900 python3 barpinch_go.py go 2>&1 | grep -v "^\["

# openrua op 161
cat > barpinch_pre.py <<'EOF'
import numpy as np, rob, sys
P=np.load("barpinch_pose.npy"); H=P[:3]; quat=list(P[3:7]); zh=P[7:10]
r=rob.Robot(); q_grasp=[-0.567,1.23,-0.11,-0.742,-0.001,1.389,1.376]
cands={"back5":H-0.05*zh,"back3":H-0.03*zh,"up6":H+np.array([0,0,0.06]),"up4back3":H-0.03*zh+np.array([0,0,0.04]),"up8":H+np.array([0,0,0.08])}
for k,p in cands.items():
    try: q=r.ik(list(p),quat,seed=q_grasp,timeout=3,avoid=True)
    except Exception as e: q=None
    print(k,np.round(p,3),None if q is None else np.round(q,3))
EOF
timeout 300 python3 barpinch_pre.py 2>&1 | grep -v "^\["

# openrua op 162
sed -n 295,320p rob.py

# openrua op 163
cat > barpinch_exec.py <<'EOF'
import numpy as np, rob, sys, subprocess
P=np.load("barpinch_pose.npy"); H=P[:3]; quat=list(P[3:7]); zh=P[7:10]
r=rob.Robot(); step=sys.argv[1]
if step=="pre":
    r.gripper(0.04); print("fingers",r.fingers())
    q_up=[-0.563,1.112,-0.116,-0.752,-0.007,1.282,1.365]
    q_up=r.ik(list(H+np.array([0,0,0.08])),quat,seed=q_up,timeout=5,avoid=True); print("q_up",np.round(q_up,3))
    ok=r.plan_exec(list(q_up),vel=0.3,acc=0.3,planning_time=10.0,time_scale=2.0)
    print("plan_exec:",ok,"q now",np.round(r.arm_q(),3)); p,_=r.fk(); print("hand",np.round(p,3),"target",np.round(H+[0,0,0.08],3))
elif step=="down":
    wps=[(list(H+np.array([0,0,dz])),quat) for dz in (0.05,0.03,0.015,0.0)]
    qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.0,max_jump=0.5,avoid=False)
    print("cart ok" if qs is not None else "cart FAILED"); p,qq=r.fk(); print("hand",np.round(p,3),"target",np.round(H,3),"quat err",np.round(np.abs(np.array(qq)-np.array(quat)).max(),3))
elif step=="close":
    r.gripper(0.0); import time
    for i in range(5): r.spin(0.2)
    print("fingers",r.fingers())
EOF
timeout 600 python3 barpinch_exec.py pre 2>&1 | grep -v "^\["

# openrua op 164
cat > mj.py <<'EOF'
import numpy as np, rob, sys, json
r=rob.Robot(); q=json.loads(sys.argv[1]); T=float(sys.argv[2]) if len(sys.argv)>2 else 4.0
print("before",np.round(r.arm_q(),3))
code,err=r.move_joints(q,T); print("code",code,"err",round(err,4)); print("after",np.round(r.arm_q(),3)); p,_=r.fk(); print("hand",np.round(p,3))
EOF
timeout 120 python3 mj.py "[-0.563,1.112,-0.116,-0.752,-0.007,1.282,1.365]" 5 2>&1 | grep -v "^\["

# openrua op 165
timeout 300 python3 barpinch_exec.py down 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview av.png >/dev/null 2>&1; echo snapped

# openrua op 166
python3 tools/perception/cam_snap.py frontview fv.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview sv.png >/dev/null 2>&1; echo ok

# openrua op 167
timeout 60 python3 cloud_any.py robot0_eye_in_hand 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
P=np.load("robot0_eye_in_hand_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
B=np.load("barpinch_pose.npy"); H=B[:3]; zh=B[7:10]; yh=B[10:13]; xh=np.cross(yh,zh)
d=P-H; x=d@xh; y=d@yh; z=d@zh
# points in the hand's gap region near the tips
m=(np.abs(x)<0.02)&(np.abs(y)<0.06)&(z>0.06)&(z<0.13)
print("pts in slab",m.sum())
ys=y[m]; zs=z[m]; xs=x[m]
H2,ye,ze=np.histogram2d(ys,zs,bins=[np.arange(-0.06,0.061,0.01),np.arange(0.06,0.131,0.01)])
print("rows y from -0.06 to 0.06 step 0.01; cols z 0.06..0.13")
for i,row in enumerate(H2): print(f"y {ye[i]:+.2f}: "+" ".join(f"{int(v):3d}" for v in row))
EOF

# openrua op 168
timeout 120 python3 barpinch_exec.py close 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png >/dev/null 2>&1

# openrua op 169
cat > lift.py <<'EOF'
import numpy as np, rob, sys
r=rob.Robot(); dz=float(sys.argv[1]); n=int(sys.argv[2]) if len(sys.argv)>2 else 3
p,q=r.fk(); print("hand",np.round(p,3),"fingers",r.fingers())
wps=[(list(p+np.array([0,0,dz*(i+1)/n])),list(q)) for i in range(n)]
qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.0,max_jump=0.5,avoid=False)
print("ok" if qs is not None else "FAILED","fingers",r.fingers())
EOF
timeout 200 python3 lift.py 0.03 2 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png >/dev/null 2>&1

# openrua op 170
timeout 300 python3 lift.py 0.15 3 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview fv2.png >/dev/null 2>&1

# openrua op 171
timeout 300 python3 lift.py 0.08 2 2>&1 | grep -v "^\["; python3 tools/perception/cam_snap.py frontview fv2.png >/dev/null 2>&1

# openrua op 172
cat > reorient.py <<'EOF'
import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q=np.array(q)  # quat x,y,z,w
R0=Rot.from_quat(q); M0=R0.as_matrix(); zh0=M0[:,2]; xh0=M0[:,0]
tip0=p+0.1034*zh0; print("hand",np.round(p,3),"tip",np.round(tip0,3),"xh",np.round(xh0,3),"zh",np.round(zh0,3),"fingers",r.fingers())
zt=np.array([0,1.0,0]); xt=np.array([0,0,1.0]) if xh0[2]>0 else np.array([0,0,-1.0]); yt=np.cross(zt,xt)
Mt=np.stack([xt,yt,zt],1); Rt=Rot.from_matrix(Mt); print("target xh",xt,"yh",yt)
tipt=np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv)>3 else np.array([-0.135,-0.42,1.09])
n=int(sys.argv[4]) if len(sys.argv)>4 else 6
sl=Slerp([0,1],Rot.concatenate([R0,Rt])); wps=[]
for i in range(1,n+1):
    s=i/n; Ri=sl([s])[0]; zi=Ri.as_matrix()[:,2]; tipi=tip0+(tipt-tip0)*s
    wps.append((list(tipi-0.1034*zi),list(Ri.as_quat())))
if len(sys.argv)>5 and sys.argv[5]=="go":
    qs=r.cart_path(wps,seconds_per_m=20.0,min_step_t=2.0,max_jump=0.6,avoid=False)
    print("ok" if qs is not None else "FAILED","fingers",r.fingers()); p,q=r.fk(); print("hand",np.round(p,3),"R",np.round(Rot.from_quat(q).as_matrix(),2))
else:
    seed=r.arm_q()
    for pos,quat in wps:
        try: qq=r.ik(pos,quat,seed=seed,timeout=3,avoid=False)
        except Exception: qq=None
        print(np.round(pos,3), None if qq is None else np.round(qq,2)); 
        if qq is not None: seed=qq
EOF
timeout 300 python3 reorient.py -0.135 -0.42 1.09 6 2>&1 | grep -v "^\["

# openrua op 173
timeout 300 python3 reorient.py -0.135 -0.42 1.09 6 2>&1 | grep -v "^\[INFO\|^\[WARN" | tail -20

# openrua op 174
cat > iktest.py <<'EOF'
import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q0=r.arm_q()
print("cur q",np.round(q0,2))
try: print("ik of current:",np.round(r.ik(list(p),list(q),seed=q0,timeout=3,avoid=False),2))
except Exception as e: print("ik current failed",e)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(3)
# target: hand pointing +y, xh=-Z, at several positions
Mt=np.stack([[0,0,-1.0],[-1.0,0,0],[0,1.0,0]],1); qt=list(Rot.from_matrix(Mt).as_quat())
for pos in ([-0.135,-0.523,1.09],[-0.135,-0.50,1.10],[-0.10,-0.50,1.10],[-0.135,-0.48,1.15],[-0.135,-0.45,1.20]):
    sols=[]
    for k in range(15):
        try: qq=r.ik(pos,qt,seed=list(rng.uniform(lo,hi)),timeout=0.5,avoid=False)
        except Exception: qq=None
        if qq is not None: sols.append(np.round(qq,2))
    print(pos,len(sols),sols[:2])
EOF
timeout 300 python3 iktest.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 175
cat > iktest2.py <<'EOF'
import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot
r=rob.Robot()
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(4)
for th in (0,10,15,20,25):
    t=np.radians(th); zh=np.array([0,np.cos(t),-np.sin(t)]); xh=np.array([0,-np.sin(t),-np.cos(t)]); yh=np.cross(zh,xh)
    qt=list(Rot.from_matrix(np.stack([xh,yh,zh],1)).as_quat())
    for x in (-0.185,):
      for y in (-0.55,-0.50,-0.45,-0.42):
        zo=1.08+0.1034*np.sin(t); pos=[x,y,zo]; sols=[]
        for k in range(12):
            try: qq=r.ik(pos,qt,seed=list(rng.uniform(lo,hi)),timeout=0.4,avoid=False)
            except Exception: qq=None
            if qq is not None: sols.append(np.round(qq,2))
        print(th,np.round(pos,3),len(sols),sols[0] if sols else "",flush=True)
EOF
timeout 600 python3 iktest2.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 176
sed -i 's/for x in (-0.185,):/for x in (-0.14,):/; s/for th in (0,10,15,20,25):/for th in (15,20,25,30):/; s/for y in (-0.55,-0.50,-0.45,-0.42):/for y in (-0.52,-0.50,-0.47,-0.44,-0.42):/' iktest2.py && timeout 600 python3 iktest2.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 177
cat > reorient2.py <<'EOF'
import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q=np.array(q); R0=Rot.from_quat(q); M0=R0.as_matrix(); zh0=M0[:,2]; xh0=M0[:,0]
tip0=p+0.1034*zh0
th=np.radians(float(sys.argv[1])); tipt=np.array([float(v) for v in sys.argv[2:5]]); n=int(sys.argv[5]); go=len(sys.argv)>6 and sys.argv[6]=="go"
zt=np.array([0,np.cos(th),-np.sin(th)]); xt=np.array([0,-np.sin(th),-np.cos(th)])
if xh0@xt<0: xt=-xt
yt=np.cross(zt,xt); Rt=Rot.from_matrix(np.stack([xt,yt,zt],1))
print("tip0",np.round(tip0,3),"->",tipt,"xh0",np.round(xh0,3),"xt",np.round(xt,3))
sl=Slerp([0,1],Rot.concatenate([R0,Rt])); wps=[]
for i in range(1,n+1):
    s=i/n; Ri=sl([s])[0]; zi=Ri.as_matrix()[:,2]; tipi=tip0+(tipt-tip0)*s
    wps.append((list(tipi-0.1034*zi),list(Ri.as_quat())))
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(5)
seed=r.arm_q(); allok=True; qs=[]
for pos,quat in wps:
    qq=None
    for k in range(8):
        sd=seed if k==0 else list(rng.uniform(lo,hi))
        try: qq=r.ik(pos,quat,seed=sd,timeout=0.5,avoid=False)
        except Exception: qq=None
        if qq is not None: break
    print(np.round(pos,3), None if qq is None else (np.round(qq,2), "jump",round(float(np.abs(np.array(qq)-np.array(seed)).max()),2)))
    if qq is None: allok=False
    else: seed=qq; qs.append(qq)
print("all reachable:",allok)
if go and allok:
    res=r.cart_path(wps,seconds_per_m=20.0,min_step_t=1.5,max_jump=0.8,avoid=False)
    print("exec", "ok" if res is not None else "FAILED", "fingers", r.fingers()); p,q=r.fk(); print("hand",np.round(p,3),"tip",np.round(p+0.1034*Rot.from_quat(q).as_matrix()[:,2],3))
EOF
timeout 300 python3 reorient2.py 10 -0.14 -0.37 1.075 8 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 178
cat > iksanity.py <<'EOF'
import numpy as np, rob
r=rob.Robot(); p,q=r.fk(); q0=r.arm_q(); print("q0",np.round(q0,3),"p",np.round(p,3))
for dz in (0,-0.01,-0.03):
    pos=list(np.array(p)+[0,0,dz])
    try: s=r.ik(pos,q,seed=q0,timeout=1.0,avoid=False)
    except Exception as e: s="EXC "+str(e)[:80]
    print(dz, None if s is None else np.round(s,3) if not isinstance(s,str) else s)
    try: s=r.ik(pos,q,seed=q0,timeout=1.0,avoid=True)
    except Exception as e: s="EXC "+str(e)[:80]
    print(" avoid", None if s is None else np.round(s,3) if not isinstance(s,str) else s)
EOF
timeout 120 python3 iksanity.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 179
timeout 60 python3 cloud_any.py birdview 2>&1 | grep -v "^\[INFO\|^\[WARN" ; python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
# points near hanging mug region, below hand housing
m=(P[:,0]>-0.30)&(P[:,0]<0.05)&(P[:,1]>-0.60)&(P[:,1]<-0.33)&(P[:,2]>0.91)&(P[:,2]<1.10)
Q=P[m]; print("n",len(Q))
# histogram in xy grid 1cm, report max z per cell
xs=np.arange(-0.30,0.05,0.01); ys=np.arange(-0.60,-0.33,0.01)
import collections
grid=np.full((len(ys),len(xs)),np.nan)
for x,y,z in Q:
    i=int((y+0.60)/0.01); j=int((x+0.30)/0.01)
    if 0<=i<len(ys) and 0<=j<len(xs): grid[i,j]=np.nanmax([grid[i,j],z])
np.set_printoptions(linewidth=250)
print("cols x from -0.30 to 0.04 (1cm); rows y from -0.60 to -0.34; value = max z*100-90 (int), '.'=none")
for i in range(len(ys)):
    row="".join("." if np.isnan(v) else (chr(ord('a')+int(min(25,max(0,(v-0.90)*100)))) ) for v in grid[i])
    print(f"{ys[i]:6.2f} {row}")
EOF

# openrua op 180
for c in frontview sideview; do timeout 60 python3 cloud_any.py $c 2>&1 | grep -v "^\[INFO\|^\[WARN"; done; python3 - <<'EOF'
import numpy as np
for cam in ("frontview","sideview"):
    P=np.load(f"{cam}_world.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.26)&(P[:,0]<0.0)&(P[:,1]>-0.56)&(P[:,1]<-0.33)&(P[:,2]>0.91)&(P[:,2]<1.10)
    Q=P[m]; print(cam,"n",len(Q))
    if len(Q)<50: continue
    # side projection: rows z (1cm) cols y (1cm), value x
    ys=np.arange(-0.56,-0.33,0.01); zs=np.arange(0.91,1.10,0.01)
    grid=np.full((len(zs),len(ys)),np.nan); cnt=np.zeros_like(grid)
    for x,y,z in Q:
        i=int((z-0.91)/0.01); j=int((y+0.56)/0.01)
        if 0<=i<len(zs) and 0<=j<len(ys): grid[i,j]=np.nanmin([grid[i,j],x]); cnt[i,j]+=1
    print(" rows z 1.09..0.91 top->bottom, cols y -0.56..-0.34; letter = min x: a=-0.26 ... (1cm steps)")
    for i in range(len(zs)-1,-1,-1):
        row="".join("." if np.isnan(v) else chr(ord('a')+int(min(25,max(0,(v+0.26)*100)))) for v in grid[i])
        print(f"{zs[i]:5.2f} {row}")
    # top projection of same points: rows y, cols x, value max z
    xs=np.arange(-0.26,0.0,0.01)
    g2=np.full((len(ys),len(xs)),np.nan)
    for x,y,z in Q:
        i=int((y+0.56)/0.01); j=int((x+0.26)/0.01)
        if 0<=i<len(ys) and 0<=j<len(xs): g2[i,j]=np.nanmax([g2[i,j],z])
    print(" top view rows y -0.56..-0.34, cols x -0.26..-0.01; letter=max z (a=0.91)")
    for i in range(len(ys)):
        print(f"{ys[i]:6.2f} "+"".join("." if np.isnan(v) else chr(ord('a')+int(min(25,max(0,(v-0.91)*100)))) for v in g2[i]))
EOF

# openrua op 181
timeout 60 python3 tools/perception/cam_snap.py frontview front.png 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 60 python3 tools/perception/cam_snap.py sideview side.png 2>&1 | grep -v "^\[INFO\|^\[WARN"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih.png 2>&1 | grep -v "^\[INFO\|^\[WARN"; ls -la front.png side.png eih.png
