#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -iv parameter; echo ---; ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id" | sort | uniq -c; echo ---; timeout 10 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id"; echo ---; ros2 node info /robot_bridge 2>/dev/null | head -60; ros2 param list /robot_bridge 2>/dev/null

# openrua op 6
timeout 20 ros2 topic echo /tf --once 2>/dev/null | grep -A 12 -E "child_frame_id: (panda_link0|birdview_optical_frame|agentview_optical_frame|robot0_eye_in_hand_optical_frame)" ; timeout 10 ros2 topic echo /birdview/color/camera_info --once | grep -A 10 "^k:"

# openrua op 7
timeout 20 ros2 topic echo /tf --once 2>/dev/null > /tmp/tf.txt; python3 - <<'EOF'
import yaml,re
txt=open('/tmp/tf.txt').read()
for blk in txt.split('- header:')[1:]:
    child=re.search(r'child_frame_id: (\S+)',blk).group(1)
    nums=re.findall(r'[xyzw]: (-?[\d.e-]+)',blk)
    print(child, nums)
EOF

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 - <<'EOF'
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
print(d.shape, np.nanmin(d), np.nanmax(d))
# table top depth
print('center depth', d[300,320], 'plate L', d[285,250], 'plate R', d[285,392], 'white mug', d[262,283], 'yellow', d[275,345], 'red', d[240,325])
# print depth around plates and mugs region
f=579.4112549695428; cx,cy=320,240
def w(u,v):
    z=d[v,u]; return np.array([-0.2+(v-cy)*z/f, (u-cx)*z/f, 3.0-z])
for name,(u,v) in dict(plateL=(250,285),plateR=(392,285),white=(283,262),yellow=(345,275),red=(325,240),table=(320,330)).items():
    print(name, w(u,v).round(3))
EOF

# openrua op 9
python3 - <<'EOF'
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
f=579.4112549695428; cx,cy=320,240
zw=3.0-d
mask=(zw>0.44)&(zw<0.72)  # above table, below arm
mask=mask.astype(np.uint8)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<30: continue
    m=lab==i
    vs,us=np.nonzero(m)
    zs=zw[m]
    # world coords using per-pixel depth
    X=-0.2+(vs-cy)*d[m]/f; Y=(us-cx)*d[m]/f
    col=img[m].mean(0)
    print(f"comp {i}: area={stats[i,4]} u={us.mean():.0f} v={vs.mean():.0f} X=[{X.min():.3f},{X.max():.3f}] Y=[{Y.min():.3f},{Y.max():.3f}] zmax={zs.max():.3f} zmed={np.median(zs):.3f} bgr={col.round(0)}")
EOF

# openrua op 10
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_depth.npy'); zw=3.0-d
f=579.4112549695428
# sample along a row v=330 (x world ~0.2) from u=200..440
row=zw[330,200:440:20]; print(row.round(3))
col=zw[150:400:20,320]; print(col.round(3))
# histogram of table region
reg=zw[200:380,150:500]; h,e=np.histogram(reg[(reg>0.4)&(reg<0.47)],bins=14,range=(0.40,0.47)); print(list(zip(e.round(3),h)))
EOF

# openrua op 11
for p in "95 295" "563 300" "205 345" "320 400"; do timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 12
python3 - <<'EOF'
import numpy as np, cv2
d=np.load('birdview_depth.npy'); img=cv2.imread('birdview.png')
f=579.4112549695428; cx,cy=320,240
zw=3.0-d
for name,(u0,u1,v0,v1) in dict(white=(260,310,245,285), yellow=(330,375,260,300), red=(305,350,220,255)).items():
    sub=zw[v0:v1,u0:u1]
    for thr in (0.48,0.50,0.52,0.54):
        m=sub>thr
        if m.sum()==0: continue
        vs,us=np.nonzero(m); vs=vs+v0; us=us+u0
        dep=d[vs,us]
        X=-0.2+(vs-cy)*dep/f; Y=(us-cx)*dep/f
        print(f"{name} z>{thr}: n={m.sum()} X=[{X.min():.3f},{X.max():.3f}] c={X.mean():.3f}  Y=[{Y.min():.3f},{Y.max():.3f}] c={Y.mean():.3f} zmax={sub[m].max():.3f}")
# hand region
sub=zw[270:300,300:345]; print('hand zmax', sub.max())
EOF

# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable control helpers for the Panda (clients built once per process)."""
import sys, time, math
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from /tf world->panda_link0
TCP_OFF = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand z pointing to world -z, fingers axis (hand y) rotated by yaw about world z.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # quaternion product Rz * Rx : (w1,x1,y1,z1)*(w2,x2,y2,z2)
    w1, x1, y1, z1 = cz, 0.0, 0.0, sz
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Ctl(Node):
    def __init__(self):
        super().__init__("ctl")
        self.js = None
        self.wrench = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._w_cb, 1)
        self.fjt = ActionClient(self, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self, GripperCommand, GRIP["port"])
        self.ik = self.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)

    def _js_cb(self, m): self.js = m
    def _w_cb(self, m): self.wrench = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def finger(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (position, R)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp_world(self):
        pos, R = self.fk_hand()
        return pos + TCP_OFF * R[:, 2]

    def ik_world(self, pos_w, quat, seed=None, tcp=True):
        """IK for hand (or TCP if tcp=True) at world position with quaternion. Returns joint list or None."""
        pos_w = np.array(pos_w, float)
        if tcp:
            R = quat_to_R(*quat)
            pos_w = pos_w - TCP_OFF * R[:, 2]
        pb = pos_w - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed if seed is not None else self.arm_q()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val} for {pos_w}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q()); err = np.abs(cur - np.array(q)).max()
        print(f"move_q code={code} maxerr={err:.4f}", flush=True)
        return code, err

    def move_path(self, qs, seconds_each=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        t = 0.0
        for q in qs:
            t += seconds_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=900)
        code = res.result().result.error_code
        cur = np.array(self.arm_q()); err = np.abs(cur - np.array(qs[-1])).max()
        print(f"move_path code={code} maxerr={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos_w, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed)
        if q is None:
            return None
        code, err = self.move_q(q, seconds)
        tcp = self.tcp_world()
        print(f"tcp now {tcp.round(4)} target {np.array(pos_w).round(4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        f = self.finger()
        print(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, v, n=20, dt=0.05):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=dt)

    def snap(self, cam, out=None):
        topic = cam if cam.startswith("/") else f"/{cam}/color/image_raw"
        out = out or f"/workspace/{cam.strip('/').split('/')[0]}.png"
        got = []
        sub = self.create_subscription(Image, topic, got.append, 1)
        while not got:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.destroy_subscription(sub)
        m = got[0]
        if "FC" in m.encoding or "16UC" in m.encoding:
            dep = self.bridge.imgmsg_to_cv2(m, "passthrough")
            np.save(out.rsplit(".", 1)[0] + ".npy", dep)
            return dep
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        cv2.imwrite(out, img)
        return img


def birdview_objects(node, tag=""):
    """Snapshot birdview color+depth, segment objects above the table, return dict of measurements."""
    img = node.snap("birdview", f"/workspace/bird{tag}.png")
    dep = node.snap("/birdview/depth/image_raw", f"/workspace/bird{tag}_d.png")
    f = 579.4112549695428; cx, cy = 320, 240
    zw = 3.0 - dep
    out = []
    mask = ((zw > 0.44) & (zw < 0.70)).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40 or stats[i, cv2.CC_STAT_AREA] > 3000: continue
        m = lab == i
        vs, us = np.nonzero(m); dd = dep[m]
        X = -0.2 + (vs - cy) * dd / f; Y = (us - cx) * dd / f
        col = img[m].mean(0)
        out.append(dict(u=us.mean(), v=vs.mean(), X=(X.min(), X.max()), Y=(Y.min(), Y.max()),
                        cx=X.mean(), cy=Y.mean(), zmax=zw[m].max(), zmed=float(np.median(zw[m])), bgr=col.round(0), area=int(m.sum())))
    for o in out:
        print(f"u={o['u']:.0f} v={o['v']:.0f} X=[{o['X'][0]:.3f},{o['X'][1]:.3f}] Y=[{o['Y'][0]:.3f},{o['Y'][1]:.3f}] c=({o['cx']:.3f},{o['cy']:.3f}) zmax={o['zmax']:.3f} zmed={o['zmed']:.3f} bgr={o['bgr']} area={o['area']}", flush=True)
    return out, img, zw
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
print("q", np.round(c.arm_q(),3))
pos,R=c.fk_hand(); print("hand world", pos.round(4)); print("R\n", R.round(3)); print("tcp", c.tcp_world().round(4))
print("fingers", c.finger())
print("down_quat(0)", down_quat(0), "R\n", quat_to_R(*down_quat(0)).round(3))
EOF

# openrua op 15
timeout 200 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
import ctl
rclpy.init(); c=Ctl()
q0=np.array(c.arm_q())
# raw FK pose (what the service returned, before adding BASE offset)
pos,R=c.fk_hand(); raw=pos-BASE_IN_WORLD
print("raw fk", raw.round(4))
# quaternion from R (x=(1,0,0,0)-like with slight tilt) -- use scipy
from scipy.spatial.transform import Rotation as Rot
quat=Rot.from_matrix(R).as_quat()
print("quat", quat.round(4))
ctl.BASE_IN_WORLD=np.zeros(3)  # test: pass raw coords straight through
for label,p in [("raw(world-like)",raw),("base-frame guess",raw-np.array([-0.51,0,0.42]))]:
    q=c.ik_world(p,quat,tcp=False)
    print(label, None if q is None else np.round(np.array(q)-q0,3))
EOF

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from /tf world->panda_link0", "new_string": "# Verified: MoveIt FK/IK poses here already are in the WORLD frame (FK of the\\n# current config matches the eye-in-hand camera TF in world), so no offset.\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 600 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.move_tcp([-0.35,0.0,1.0], down_quat(0), seconds=3.0)
print("q", np.round(q,3) if q is not None else None)
objs,img,zw=birdview_objects(c,"1")
EOF

# openrua op 18
timeout 600 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=[0.083,-1.087,-0.026,-2.22,-0.025,1.133,0.061]
for i in range(3):
    code,err=c.move_q(q,3.0)
    if err<0.02: break
print("tcp", c.tcp_world().round(4))
objs,img,zw=birdview_objects(c,"2")
EOF

# openrua op 19
python3 - <<'EOF'
import cv2, numpy as np
img=cv2.imread('/workspace/bird2.png')
crop=img[200:320,220:420]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST)
cv2.imwrite('/workspace/bird2_crop.png',crop)
EOF

# openrua op 20
python3 - <<'EOF'
import numpy as np, cv2
d=np.load('/workspace/bird2_d.npy'); zw=3.0-d
f=579.4112549695428; cx,cy=320,240
def world(us,vs):
    dd=d[vs,us]; return -0.2+(vs-cy)*dd/f, (us-cx)*dd/f
for name,(u0,u1,v0,v1) in dict(white=(262,308,243,283), yellow=(322,372,255,298), red=(300,352,218,255)).items():
    sub=zw[v0:v1,u0:u1]
    rim=sub>0.52
    vs,us=np.nonzero(rim); us=us+u0; vs=vs+v0
    # circle fit (algebraic)
    A=np.c_[2*us,2*vs,np.ones(len(us))]; b=us**2+vs**2
    sol=np.linalg.lstsq(A,b,rcond=None)[0]; uc,vc=sol[0],sol[1]; r=np.sqrt(sol[2]+uc**2+vc**2)
    X,Y=world(np.array([int(round(uc))]),np.array([int(round(vc))]))
    # interior: low pixels inside circle
    zc=zw[int(round(vc)),int(round(uc))]
    dep_c=d[int(round(vc)),int(round(uc))]
    Xc=-0.2+(vc-cy)*2.45/f; Yc=(uc-cx)*2.45/f  # rim at z~0.55 -> depth 2.45
    print(f"{name}: rim circle u={uc:.1f} v={vc:.1f} r_px={r:.1f} -> r={r*2.45/f:.3f} m ; center world ({Xc:.3f},{Yc:.3f}) ; z at center {zc:.3f} ; rim zmax {sub[rim].max():.3f}")
EOF

# openrua op 21
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np, cv2
from ctl import *
rclpy.init(); c=Ctl()
c.gripper(0.04)
yc=np.array([-0.051,0.103]); off=np.array([0.0,0.045])
p=yc+off
q=c.move_tcp([p[0],p[1],0.70], down_quat(0), 3.0)
if q is None: raise SystemExit("ik fail pregrasp")
q=c.move_tcp([p[0],p[1],0.60], down_quat(0), 2.0, seed=q)
q=c.move_tcp([p[0],p[1],0.52], down_quat(0), 2.0, seed=q)
print("q", np.round(q,3))
pos,R=c.fk_hand(); print("hand R\n",R.round(3))
c.snap("robot0_eye_in_hand","/workspace/eih_y1.png"); c.snap("agentview","/workspace/agent_y1.png")
print("wrench", c.wrench.wrench.force if c.wrench else None)
EOF

# openrua op 22
timeout 300 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=[0.199,0.237,0.106,-2.459,-0.057,2.694,0.353]
pos,R=c.fk_hand(q); print("fk(q_ik) pos",pos.round(4)); print(R.round(3))
cur=c.arm_q(); print("cur", np.round(cur,3))
pos,R=c.fk_hand(cur); print("fk(cur) pos",pos.round(4)); print(R.round(3))
# test: what does IK do for down_quat(pi/4)?
for yaw in (0, np.pi/4, -np.pi/4):
    qq=c.ik_world([-0.051,0.148,0.52], down_quat(yaw), seed=cur)
    if qq: 
        p2,R2=c.fk_hand(qq); print("yaw",yaw," q7",round(qq[6],3),"R col1",R2[:,1].round(3))
EOF

# openrua op 23
timeout 600 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
f=c.gripper(0.0)
print("wrench", c.wrench.wrench.force)
c.snap("robot0_eye_in_hand","/workspace/eih_y2.png")
EOF

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "    def move_q(self, q, seconds=3.0, tol=0.02):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_q(self, q, seconds=3.0, tol=0.02, retries=3):\\n        # controller lag often yields error_code -5 short of the goal; resend converges\\n        for i in range(retries):\\n            code, err = self._move_q_once(q, seconds)\\n            if err < tol:\\n                break\\n        return code, err\\n\\n    def _move_q_once(self, q, seconds):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
cur=c.arm_q()
# lift straight up keeping current orientation: use IK with the current hand quaternion
from scipy.spatial.transform import Rotation as Rot
pos,R=c.fk_hand()
# link8 orientation = hand rotated by +45deg about z? just request down_quat(0) which gave this hand yaw
q=c.move_tcp([-0.057,0.146,0.75], down_quat(0), 3.0, seed=cur)
print("fingers", c.finger(), "wrench", c.wrench.wrench.force)
c.snap("agentview","/workspace/agent_y3.png")
EOF

# openrua op 26
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q()
q=c.move_tcp([-0.03,-0.05,0.80], down_quat(0), 3.0, seed=q)
q=c.move_tcp([-0.014,-0.26,0.80], down_quat(0), 3.0, seed=q)
print("fingers", c.finger())
objs,img,zw=birdview_objects(c,"3")
EOF

# openrua op 27
python3 - <<'EOF'
import numpy as np, cv2
d=np.load('/workspace/bird3_d.npy'); zw=3.0-d
f=579.4112549695428; cx,cy=320,240
m=(zw>0.66)&(zw<0.795)
vs,us=np.nonzero(m); print("n",len(us))
dd=d[vs,us]; X=-0.2+(vs-cy)*dd/f; Y=(us-cx)*dd/f
print("X range",X.min(),X.max(),"Y range",Y.min(),Y.max())
# rim only (top band)
mr=(zw>0.75)&(zw<0.795); vs,us=np.nonzero(mr)
A=np.c_[2*us,2*vs,np.ones(len(us))]; b=us**2+vs**2
sol=np.linalg.lstsq(A,b,rcond=None)[0]; uc,vc=sol[0],sol[1]; r=np.sqrt(sol[2]+uc**2+vc**2)
dep=3.0-0.77
print(f"rim circle center world ({-0.2+(vc-cy)*dep/f:.3f},{(uc-cx)*dep/f:.3f}) r={r*dep/f:.3f}")
img=cv2.imread('/workspace/bird3.png'); crop=cv2.resize(img[220:320,200:320],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST); cv2.imwrite('/workspace/bird3_crop.png',crop)
EOF

# openrua op 28
timeout 300 python3 -u - <<'EOF'
import rclpy, numpy as np, cv2
from ctl import *
rclpy.init(); c=Ctl()
img=c.snap("robot0_eye_in_hand","/workspace/eih_y4.png")
dep=c.snap("/robot0_eye_in_hand/depth/image_raw","/workspace/eih_y4_d.png")
c.snap("agentview","/workspace/agent_y4.png")
c.snap("sideview","/workspace/side_y4.png")
print(dep.shape, np.nanmin(dep), np.nanmax(dep))
EOF
timeout 10 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 29
timeout 120 ros2 topic echo /tf --once 2>/dev/null > /tmp/tf2.txt; python3 - <<'EOF'
import re, numpy as np, cv2
txt=open('/tmp/tf2.txt').read()
for blk in txt.split('- header:')[1:]:
    child=re.search(r'child_frame_id: (\S+)',blk).group(1)
    if child!='robot0_eye_in_hand_optical_frame': continue
    nums=[float(x) for x in re.findall(r'[xyzw]: (-?[\d.e-]+)',blk)]
t=np.array(nums[:3]); x,y,z,w=nums[3:]
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("cam pos",t.round(3)); print(R.round(3))
d=np.load('/workspace/eih_y4_d.npy'); img=cv2.imread('/workspace/eih_y4.png')
fx=312.77408948188935; cx,cy=320,240
vs,us=np.mgrid[0:480,0:640]
P=np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t
P=P.reshape(480,640,3)
Z=P[...,2]
# plate: z in 0.44..0.47, whitish
white=(img.min(-1)>150)
pm=(Z>0.435)&(Z<0.47)&white
vs2,us2=np.nonzero(pm); print("plate px",len(us2)); 
if len(us2): print("plate pts X",P[pm][:,0].min().round(3),P[pm][:,0].max().round(3),"Y",P[pm][:,1].min().round(3),P[pm][:,1].max().round(3))
# mug rim: Z in 0.74..0.79
mm=(Z>0.73)&(Z<0.795)
print("mug px",mm.sum())
if mm.sum():
    pts=P[mm]; print("mug X",pts[:,0].min().round(3),pts[:,0].max().round(3),"Y",pts[:,1].min().round(3),pts[:,1].max().round(3))
    # circle fit in XY
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
    s=np.linalg.lstsq(A,b,rcond=None)[0]; print("rim circle center",s[0].round(3),s[1].round(3),"r",np.sqrt(s[2]+s[0]**2+s[1]**2).round(3))
# mug body band
mb=(Z>0.68)&(Z<0.73); pts=P[mb]; print("body X",pts[:,0].min().round(3),pts[:,0].max().round(3),"Y",pts[:,1].min().round(3),pts[:,1].max().round(3))
EOF

# openrua op 30
python3 - <<'EOF'
import re, numpy as np, cv2
txt=open('/tmp/tf2.txt').read()
for blk in txt.split('- header:')[1:]:
    child=re.search(r'child_frame_id: (\S+)',blk).group(1)
    if child!='robot0_eye_in_hand_optical_frame': continue
    nums=[float(x) for x in re.findall(r'[xyzw]: (-?[\d.e-]+)',blk)]
t=np.array(nums[:3]); x,y,z,w=nums[3:]
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
d=np.load('/workspace/eih_y4_d.npy'); img=cv2.imread('/workspace/eih_y4.png')
fx=312.77408948188935; cx,cy=320,240
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t).reshape(480,640,3)
Z=P[...,2]
for lo,hi in [(0.72,0.75),(0.75,0.78),(0.78,0.81),(0.81,0.85)]:
    m=(Z>lo)&(Z<hi); pts=P[m]
    if len(pts)==0: print(lo,hi,"none"); continue
    vs2,us2=np.nonzero(m)
    print(f"Z {lo}-{hi}: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] u[{us2.min()},{us2.max()}] v[{vs2.min()},{vs2.max()}]")
# Save a mask image of Z bands for viewing
vis=np.zeros((480,640,3),np.uint8)
vis[(Z>0.72)&(Z<0.75)]=(255,0,0); vis[(Z>0.75)&(Z<0.80)]=(0,255,0); vis[(Z>0.80)&(Z<0.86)]=(0,0,255); vis[(Z>0.43)&(Z<0.47)&(img.min(-1)>150)]=(255,255,255)
cv2.imwrite('/workspace/eih_y4_bands.png',vis)
EOF

# openrua op 31
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q()
q=c.move_tcp([-0.037,-0.275,0.62], down_quat(0), 3.0, seed=q)
q=c.move_tcp([-0.037,-0.275,0.56], down_quat(0), 2.0, seed=q)
print("fingers", c.finger(), "wrench", c.wrench.wrench.force)
c.snap("robot0_eye_in_hand","/workspace/eih_y5.png"); c.snap("/robot0_eye_in_hand/depth/image_raw","/workspace/eih_y5_d.png")
c.snap("agentview","/workspace/agent_y5.png")
EOF
timeout 60 ros2 topic echo /tf --once 2>/dev/null > /tmp/tf3.txt

# openrua op 32
cat > /workspace/eih_measure.py <<'EOF'
"""Measure objects from an eye-in-hand color+depth pair using the TF dump: prints XY extents per Z band."""
import re, sys, numpy as np, cv2
tfpath, dpath, ipath = sys.argv[1:4]
txt=open(tfpath).read()
for blk in txt.split('- header:')[1:]:
    child=re.search(r'child_frame_id: (\S+)',blk).group(1)
    if child!='robot0_eye_in_hand_optical_frame': continue
    nums=[float(x) for x in re.findall(r'[xyzw]: (-?[\d.e-]+)',blk)]
t=np.array(nums[:3]); x,y,z,w=nums[3:]
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
print("cam", t.round(4))
d=np.load(dpath); img=cv2.imread(ipath)
fx=312.77408948188935; cx,cy=320,240
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fx,d],-1).reshape(-1,3)@R.T+t).reshape(480,640,3)
Z=P[...,2]
bands=[tuple(map(float,b.split(':'))) for b in sys.argv[4:]]
for lo,hi in bands:
    m=(Z>lo)&(Z<hi); pts=P[m]
    if len(pts)==0: print(lo,hi,"none"); continue
    vs2,us2=np.nonzero(m)
    print(f"Z {lo}-{hi}: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] u[{us2.min()},{us2.max()}] v[{vs2.min()},{vs2.max()}]")
# plate (white, low)
pm=(Z>0.43)&(Z<0.47)&(img.min(-1)>150); pts=P[pm]
if len(pts): print(f"plate-white: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
np.save('/workspace/last_P.npy',P)
EOF
python3 /workspace/eih_measure.py /tmp/tf3.txt /workspace/eih_y5_d.npy /workspace/eih_y5.png 0.48:0.51 0.51:0.54 0.54:0.57

# openrua op 33
wc -l /tmp/tf3.txt; grep child_frame_id /tmp/tf3.txt; for i in 1 2 3; do timeout 60 ros2 topic echo /tf --once 2>/dev/null > /tmp/tf3.txt; grep -q eye_in_hand /tmp/tf3.txt && break; done; grep child_frame_id /tmp/tf3.txt | head -3; python3 /workspace/eih_measure.py /tmp/tf3.txt /workspace/eih_y5_d.npy /workspace/eih_y5.png 0.48:0.51 0.51:0.54 0.54:0.57

# openrua op 34
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('/workspace/last_P.npy'); img=cv2.imread('/workspace/eih_y5.png')
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
reg=(Y<-0.2)&(X>-0.1)&(X<0.15)
def circ(pts):
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
    s=np.linalg.lstsq(A,b,rcond=None)[0]; return s[0],s[1],np.sqrt(s[2]+s[0]**2+s[1]**2)
# plate: all points at plate height in region (incl. rings)
pm=reg&(Z>0.44)&(Z<0.465)
pts=P[pm]; print("plate pts",len(pts),"X",pts[:,0].min().round(3),pts[:,0].max().round(3),"Y",pts[:,1].min().round(3),pts[:,1].max().round(3))
# plate edge: boundary pixels of plate mask (outer)
# use points at the plate rim height (higher part: 0.452-0.462) 
pr=reg&(Z>0.452)&(Z<0.465); pts=P[pr]; print("plate rim pts",len(pts), "circle", np.round(circ(pts),3))
for lo,hi in [(0.48,0.50),(0.50,0.53),(0.53,0.56),(0.56,0.59)]:
    m=reg&(Z>lo)&(Z<hi); pts=P[m]
    if len(pts)<10: print(lo,hi,'none'); continue
    print(f"mug Z {lo}-{hi}: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}]")
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('/workspace/last_P.npy'); img=cv2.imread('/workspace/eih_y5.png')
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
reg=(Y<-0.215)&(X>-0.06)&(X<0.15)
for lo,hi in [(0.485,0.50),(0.50,0.53),(0.53,0.56),(0.56,0.59)]:
    m=reg&(Z>lo)&(Z<hi); pts=P[m]
    if len(pts)<10: print(lo,hi,'none'); continue
    vs,us=np.nonzero(m)
    print(f"mug Z {lo}-{hi}: n={len(pts)} X[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] Y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] u[{us.min()},{us.max()}] v[{vs.min()},{vs.max()}]")
vis=np.zeros((480,640,3),np.uint8)
vis[reg&(Z>0.485)&(Z<0.53)]=(255,0,0); vis[reg&(Z>0.53)&(Z<0.59)]=(0,255,0); vis[(Z>0.44)&(Z<0.465)]=(255,255,255)
cv2.imwrite('/workspace/eih_y5_bands.png',vis)
EOF

# openrua op 36
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q()
q=c.move_tcp([-0.037,-0.275,0.535], down_quat(0), 2.0, seed=q)
print("wrench", c.wrench.wrench.force)
c.gripper(0.04)
q=c.move_tcp([-0.037,-0.275,0.70], down_quat(0), 2.5, seed=q)
q=c.move_tcp([-0.25,-0.15,0.95], down_quat(0), 3.0, seed=q)
objs,img,zw=birdview_objects(c,"4")
c.snap("agentview","/workspace/agent_y6.png")
EOF

# openrua op 37
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q()
th=np.pi/4
p=[-0.065,-0.154]
q=c.move_tcp([p[0],p[1],0.70], down_quat(th), 3.0, seed=q)
if q is None: raise SystemExit("ik fail")
pos,R=c.fk_hand(); print("finger axis", R[:,1].round(3))
q=c.move_tcp([p[0],p[1],0.60], down_quat(th), 2.0, seed=q)
q=c.move_tcp([p[0],p[1],0.515], down_quat(th), 2.0, seed=q)
print("q", np.round(q,3), "fingers", c.finger(), "wrench", c.wrench.wrench.force)
c.snap("robot0_eye_in_hand","/workspace/eih_w1.png")
EOF

# openrua op 38
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
c.gripper(0.0)
q=c.arm_q(); th=np.pi/4
q=c.move_tcp([-0.065,-0.154,0.75], down_quat(th), 3.0, seed=q)
print("fingers", c.finger(), "wrench", c.wrench.wrench.force)
c.snap("agentview","/workspace/agent_w2.png")
EOF

# openrua op 39
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q(); th=np.pi/4
q=c.move_tcp([0.0,0.08,0.80], down_quat(th), 3.0, seed=q)
q=c.move_tcp([0.039,0.312,0.80], down_quat(th), 3.0, seed=q)
q=c.move_tcp([0.039,0.312,0.62], down_quat(th), 2.5, seed=q)
q=c.move_tcp([0.039,0.312,0.56], down_quat(th), 2.0, seed=q)
print("fingers", c.finger(), "wrench", c.wrench.wrench.force)
c.snap("robot0_eye_in_hand","/workspace/eih_w3.png"); c.snap("/robot0_eye_in_hand/depth/image_raw","/workspace/eih_w3_d.png")
c.snap("agentview","/workspace/agent_w3.png")
EOF
for i in 1 2 3; do timeout 60 ros2 topic echo /tf --once 2>/dev/null > /tmp/tf4.txt; grep -q eye_in_hand /tmp/tf4.txt && break; done
python3 /workspace/eih_measure.py /tmp/tf4.txt /workspace/eih_w3_d.npy /workspace/eih_w3.png 0.485:0.50 0.50:0.53 0.53:0.56

# openrua op 40
python3 - <<'EOF'
import numpy as np, cv2
P=np.load('/workspace/last_P.npy'); img=cv2.imread('/workspace/eih_w3.png')
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
def circ(pts):
    A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=pts[:,0]**2+pts[:,1]**2
    s=np.linalg.lstsq(A,b,rcond=None)[0]; return s[0],s[1],np.sqrt(s[2]+s[0]**2+s[1]**2)
reg=(Y>0.2)
pr=reg&(Z>0.452)&(Z<0.465); pts=P[pr]; print("plate rim pts",len(pts), "circle", np.round(circ(pts),3), "X",pts[:,0].min().round(3),pts[:,0].max().round(3),"Y",pts[:,1].min().round(3),pts[:,1].max().round(3))
vis=np.zeros((480,640,3),np.uint8)
vis[reg&(Z>0.485)&(Z<0.50)]=(255,0,0); vis[reg&(Z>0.50)&(Z<0.56)]=(0,255,0); vis[(Z>0.44)&(Z<0.465)]=(255,255,255)
cv2.imwrite('/workspace/eih_w3_bands.png',vis)
# handle? mug band 0.5-0.53 has Y max 0.390 vs 0.485-0.5 band Y max .347 -> handle on +y side
EOF

# openrua op 41
timeout 900 python3 -u - <<'EOF'
import rclpy, numpy as np
from ctl import *
rclpy.init(); c=Ctl()
q=c.arm_q(); th=np.pi/4
q=c.move_tcp([0.039,0.312,0.535], down_quat(th), 2.0, seed=q)
print("wrench", c.wrench.wrench.force)
c.gripper(0.04)
q=c.move_tcp([0.039,0.312,0.70], down_quat(th), 2.5, seed=q)
q=c.move_tcp([-0.30,0.0,0.95], down_quat(th), 3.0, seed=q)
objs,img,zw=birdview_objects(c,"5")
c.snap("agentview","/workspace/agent_final.png"); c.snap("frontview","/workspace/front_final.png")
EOF

# openrua op 42
python3 -c "
import cv2; img=cv2.imread('/workspace/bird5.png'); cv2.imwrite('/workspace/bird5_crop.png', cv2.resize(img[200:320,220:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"
