#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ---ACTIONS; ros2 action list; echo ---SERVICES; ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ---NODES; ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ls *.png

# openrua op 4
ros2 node info /robot_bridge; echo ---; ros2 topic echo /pipeline_state --once 2>&1 | head -30; echo ---; ros2 topic info /pipeline_state

# openrua op 5
cat /tmp/claude-1000/-workspace/79c95790-1f87-4500-bbc5-134a2262f695/tasks/bcy0bs1he.output

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c | head -40; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child_frame_id" | sort | uniq -c

# openrua op 7
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; for p in "255 272" "378 238" "378 282" "318 310" "200 200"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
python3 -c "
import cv2
for name,(x0,y0,x1,y1) in {'agentview':(360,190,480,290),'frontview':(360,300,480,380),'sideview':(330,300,430,380),'robot0_eye_in_hand':(60,190,220,340),'birdview':(340,210,420,260)}.items():
    im=cv2.imread(name+'.png'); c=im[y0:y1,x0:x1]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_'+name+'.png',c)
"

# openrua op 9
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable robot helpers for this Panda workstation (one node, reused clients).

World frame = panda_link0 shifted by BASE (world->panda_link0 from TF).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([-0.660, 0.000, 0.912])  # world -> panda_link0 translation
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand pointing straight down (hand z = world -z), hand x rotated by yaw
    about world z. yaw=0 -> hand x = world +x, fingers close along world y."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy); product (Rz*Rx):
    # w = cy*0 - sy*0 = 0 ; x = cy*1 ; y = sy*1 ; z = 0  (check via matrices)
    return (cy, sy, 0.0, 0.0)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._w_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _js_cb(self, m):
        self.js = m

    def _w_cb(self, m):
        self.wrench = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def hand_pose(self, q=None):
        """FK: returns (pos_world[3], quat[4] xyzw) of panda_hand."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.hand_pose(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat, seed=None, at_tcp=False, timeout=60):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in (seed or self.arm_q())]
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print("move_pose: IK FAILED, no motion", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        p, _ = (self.tcp_pose() if at_tcp else self.hand_pose())
        print(f"move_pose: target={np.round(pos_world,4)} actual={np.round(p,4)}", flush=True)
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v=(0, 0, 0), w=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            self.tw_pub.publish(msg)
            self.spin(dt)
        return self.hand_pose()


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q:", np.round(q, 4))
    print("finger gap:", r.finger_gap())
    pos, quat = r.hand_pose(q)
    print("hand world pos:", np.round(pos, 4), "quat:", np.round(quat, 4))
    tp, _ = r.tcp_pose(q)
    print("tcp world pos:", np.round(tp, 4))
    # IK round trip check
    sol = r.ik(pos, quat)
    print("ik roundtrip:", None if sol is None else np.round(sol, 4))
OPENRUA_EOF

# openrua op 10
timeout 300 python3 -u rob.py 2>&1 | tail -20

# openrua op 11
sed -i 's/self.ik = self.node.create_client/self.ik_cli = self.node.create_client/; s/self.ik.wait_for_service/self.ik_cli.wait_for_service/; s/fut = self.ik.call_async(req)/fut = self.ik_cli.call_async(req)/' rob.py && timeout 300 python3 -u -c "
import rclpy, numpy as np
from rob import *
r = Robot()
q = r.arm_q()
req = GetPositionFK.Request()
req.header.frame_id = ''
req.fk_link_names = ['panda_link0','panda_link1','panda_link4','panda_link8','panda_hand']
req.robot_state.joint_state.name = list(ARM)
req.robot_state.joint_state.position = [float(v) for v in q]
fut = r.fk.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
print(res.error_code)
for n, ps in zip(res.fk_link_names, res.pose_stamped):
    p=ps.pose.position; o=ps.pose.orientation
    print(n, ps.header.frame_id, round(p.x,4), round(p.y,4), round(p.z,4), '|', round(o.x,4), round(o.y,4), round(o.z,4), round(o.w,4))
" 2>&1 | tail -12

# openrua op 12
sed -i 's/pos = np.array(\[p.position.x, p.position.y, p.position.z\]) + BASE/pos = np.array([p.position.x, p.position.y, p.position.z])  # FK already reports world/' rob.py && timeout 300 python3 -u -c "
import numpy as np
from rob import *
r = Robot()
q = r.arm_q()
pos, quat = r.hand_pose(q)
print('hand', np.round(pos,4), np.round(quat,4))
s1 = r.ik(pos, quat)             # subtracts BASE (base-frame interpretation)
print('ik base-frame interp:', None if s1 is None else np.round(s1,4))
s2 = r.ik(pos + BASE, quat)      # no net subtraction (world interp)
print('ik world interp:', None if s2 is None else np.round(s2,4))
print('current q:', np.round(q,4))
" 2>&1 | tail -6

# openrua op 13
timeout 300 python3 -u -c "
import numpy as np
from rob import *
from scipy.spatial.transform import Rotation as Rot
Rh = Rot.from_quat([0.9996,0,-0.0284,0]); R8 = Rot.from_quat([0.9235,-0.3825,-0.0262,0.0109])
print('hand->link8 rel rotvec:', (Rh.inv()*R8).as_rotvec())
r = Robot()
q = r.arm_q()
# FK link8
req = GetPositionFK.Request(); req.fk_link_names=['panda_link8']; req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=[float(v) for v in q]
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30); p=fut.result().pose_stamped[0].pose
pos=np.array([p.position.x,p.position.y,p.position.z]); quat=(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
print('link8 world', np.round(pos,4), np.round(quat,4))
s1 = r.ik(pos, quat); print('base interp', None if s1 is None else np.round(s1,4))
s2 = r.ik(pos+BASE, quat); print('world interp', None if s2 is None else np.round(s2,4))
print('current', np.round(q,4))
" 2>&1 | tail -5

# openrua op 14
python3 - <<'EOF'
import re
s = open('/workspace/rob.py').read()
s = s.replace("""        pos = pos - BASE
        req = GetPositionIK.Request()""", """        # IK tip link is panda_link8 (= hand rotated +45deg about z); poses
        # are interpreted in the same world frame FK reports (verified).
        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)))
        req = GetPositionIK.Request()""")
s = s.replace("""def down_quat(yaw):""", """def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def down_quat(yaw):""")
s = s.replace("""    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy); product (Rz*Rx):
    # w = cy*0 - sy*0 = 0 ; x = cy*1 ; y = sy*1 ; z = 0  (check via matrices)
    return (cy, sy, 0.0, 0.0)""", """    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_mul((0.0, 0.0, sy, cy), (1.0, 0.0, 0.0, 0.0))""")
open('/workspace/rob.py','w').write(s)
EOF
timeout 300 python3 -u -c "
import numpy as np
from rob import *
from scipy.spatial.transform import Rotation as Rot
for yaw in [0, 0.5, -1.0]:
    q = down_quat(yaw); R = quat_to_R(q); print('yaw',yaw,'hand x,y,z axes:', np.round(R[:,0],3), np.round(R[:,1],3), np.round(R[:,2],3))
r = Robot()
q = r.arm_q(); pos, quat = r.hand_pose(q)
s = r.ik(pos, quat); print('roundtrip', np.round(s,4)); print('current ', np.round(q,4))
# IK for a pose above the knob
s = r.ik([-0.207, 0.204, 1.10], down_quat(0.0)); print('above knob', None if s is None else np.round(s,4))
if s is not None:
    p,qq = r.hand_pose(s); print('fk of sol', np.round(p,4), np.round(qq,4), 'want', np.round(down_quat(0.0),4))
" 2>&1 | tail -8

# openrua op 15
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world-frame point cloud
as <cam>_cloud.npy (H x W x 3) plus <cam>.png color."""
import struct
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    cv2.imwrite(f"{cam}.png", img)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d, np.ones_like(d)], -1)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 15 and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    pw = (pc.reshape(-1, 4) @ T.T)[:, :3].reshape(H, W, 3)
    np.save(f"{cam}_cloud.npy", pw)
    print(f"{cam}: {W}x{H} cloud saved; cam pos {np.round(T[:3,3],3)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 16
timeout 200 python3 -u cloud.py birdview && timeout 200 python3 -u cloud.py agentview && python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy')
z = pw[...,2]
# table height
print('table z (median of region):', np.median(z[350:400, 200:250]))
# objects above table: z > 0.905
mask = z > 0.905
import cv2
ys, xs = np.where(mask)
print('n above table', mask.sum())
# knob region (pixels 355..400, 220..255)
for name,(y0,y1,x0,x1) in {'knob':(220,256,355,400),'stove':(250,310,345,410),'pan':(240,300,220,340),'moka':(290,335,295,340)}.items():
    sub = pw[y0:y1,x0:x1]; zz = sub[...,2]; m = zz>0.905
    if m.sum()==0: print(name,'nothing'); continue
    pts = sub[m]
    print(name, 'n',m.sum(),'x[%.3f %.3f] y[%.3f %.3f] zmax %.3f zmed %.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),zz[m].max(), np.median(zz[m])))
"


# openrua op 17
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy'); z = pw[...,2]
def region(y0,y1,x0,x1,zlo=0.905,zhi=1.2):
    sub = pw[y0:y1,x0:x1]; zz=sub[...,2]; m=(zz>zlo)&(zz<zhi); return sub[m]
# knob
k = region(220,256,355,400)
print('knob all: n',len(k),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(k[:,0].min(),k[:,0].max(),k[:,1].min(),k[:,1].max(),k[:,2].min(),k[:,2].max()))
for lo in [0.91,0.93,0.95,0.97]:
    s = k[k[:,2]>lo]
    if len(s): print(' z>%.2f n=%d x[%.3f %.3f] y[%.3f %.3f] centroid %s'%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),np.round(s.mean(0),3)))
# hist of knob z
h,e = np.histogram(k[:,2], bins=np.arange(0.90,1.0,0.005)); print('knob z hist', list(zip(np.round(e[:-1],3), h)))
# stove
s = region(250,310,345,410)
print('stove: x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f] centroid %s'%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),s[:,2].min(),s[:,2].max(),np.round(s.mean(0),3)))
h,e = np.histogram(s[:,2], bins=np.arange(0.90,0.96,0.005)); print('stove z hist', list(zip(np.round(e[:-1],3), h)))
# pan
p = region(240,300,220,340, zhi=1.1)
print('pan: x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f] centroid %s'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),np.round(p.mean(0),3)))
h,e = np.histogram(p[:,2], bins=np.arange(0.90,1.02,0.01)); print('pan z hist', list(zip(np.round(e[:-1],3), h)))
"


# openrua op 18
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy')
sub = pw[230:310,200:345]; zz=sub[...,2]; m=(zz>0.905)&(zz<1.1); p=sub[m]
# body vs handle: body is the disc around center. estimate body center from points with z>1.0 (rim)
rim = p[p[:,2]>1.0]; print('rim n',len(rim),'x[%.3f %.3f] y[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()), 'center', np.round(rim.mean(0),3))
cx, cy = rim[:,0].mean(), rim[:,1].mean()
rr = np.hypot(p[:,0]-cx, p[:,1]-cy)
body = p[rr<0.11]; handle = p[rr>=0.12]
print('body z hist', np.histogram(body[:,2], bins=np.arange(0.90,1.05,0.01)))
print('handle n',len(handle),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(handle[:,0].min(),handle[:,0].max(),handle[:,1].min(),handle[:,1].max(),handle[:,2].min(),handle[:,2].max()))
# handle slices by y
for y0 in np.arange(-0.10, 0.06, 0.02):
    s = handle[(handle[:,1]>=y0)&(handle[:,1]<y0+0.02)]
    if len(s): print(' y %.2f..%.2f n=%d x[%.3f %.3f] zmax %.3f zmed %.3f'%(y0,y0+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,2].max(),np.median(s[:,2])))
# interior bottom
inner = body[rr[rr<0.11]<0.07]; print('inner z hist', np.histogram(inner[:,2], bins=np.arange(0.90,1.05,0.01)))
"


# openrua op 19
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
def sel(x0,x1,y0,y1,z0=0.905,z1=1.1):
    m=(pw[:,0]>x0)&(pw[:,0]<x1)&(pw[:,1]>y0)&(pw[:,1]<y1)&(pw[:,2]>z0)&(pw[:,2]<z1); return pw[m]
body = sel(-0.25,0.0,-0.35,-0.12)
print('body n',len(body),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(body[:,0].min(),body[:,0].max(),body[:,1].min(),body[:,1].max(),body[:,2].min(),body[:,2].max()))
rim = body[body[:,2]>1.005]; print('rim center', np.round(rim.mean(0),3), 'rim x[%.3f %.3f] y[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
handle = sel(-0.15,-0.02,-0.12,0.06)
print('handle n',len(handle),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(handle[:,0].min(),handle[:,0].max(),handle[:,1].min(),handle[:,1].max(),handle[:,2].min(),handle[:,2].max()))
for y0 in np.arange(-0.12, 0.06, 0.02):
    s = handle[(handle[:,1]>=y0)&(handle[:,1]<y0+0.02)]
    if len(s): print(' y %.2f..%.2f n=%d x[%.3f %.3f] xmean %.3f zmin %.3f zmax %.3f zmed %.3f'%(y0,y0+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,0].mean(),s[:,2].min(),s[:,2].max(),np.median(s[:,2])))
"


# openrua op 20
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.30)&(pw[:,0]<-0.10)&(pw[:,1]>0.10)&(pw[:,1]<0.30)&(pw[:,2]>0.905)&(pw[:,2]<0.93)
d=pw[m]; print('disc n',len(d),'x[%.3f %.3f] y[%.3f %.3f] center %s'%(d[:,0].min(),d[:,0].max(),d[:,1].min(),d[:,1].max(),np.round(d.mean(0),3)))
# disc center estimate: fit circle to boundary approx by min/max
print('disc center by extents:', (d[:,0].min()+d[:,0].max())/2, (d[:,1].min()+d[:,1].max())/2)
m=(pw[:,0]>-0.30)&(pw[:,0]<-0.10)&(pw[:,1]>0.10)&(pw[:,1]<0.30)&(pw[:,2]>0.94)
f=pw[m]; print('fin n',len(f),'x[%.3f %.3f] y[%.3f %.3f] center %s'%(f[:,0].min(),f[:,0].max(),f[:,1].min(),f[:,1].max(),np.round(f.mean(0),3)))
"


# openrua op 21
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.30)&(pw[:,0]<-0.166)&(pw[:,1]>0.10)&(pw[:,1]<0.30)&(pw[:,2]>0.912)&(pw[:,2]<0.93)
d=pw[m]; print('disc n',len(d),'x[%.3f %.3f] y[%.3f %.3f] center %s'%(d[:,0].min(),d[:,0].max(),d[:,1].min(),d[:,1].max(),np.round(d.mean(0),3)))
# per-y-slice x extents to find the circle
for y0 in np.arange(0.16,0.25,0.01):
    s=d[(d[:,1]>=y0)&(d[:,1]<y0+0.01)]
    if len(s): print(' y %.2f x[%.3f %.3f]'%(y0,s[:,0].min(),s[:,0].max()))
m=(pw[:,0]>-0.30)&(pw[:,0]<-0.10)&(pw[:,1]>0.10)&(pw[:,1]<0.30)&(pw[:,2]>0.94)&(pw[:,2]<1.0)
f=pw[m]; print('fin n',len(f),'x[%.3f %.3f] y[%.3f %.3f] center %s'%(f[:,0].min(),f[:,0].max(),f[:,1].min(),f[:,1].max(),np.round(f.mean(0),3)))
"


# openrua op 22
cat > step1.py <<'EOF'
import numpy as np
from rob import *
r = Robot()
r.gripper(0.04)
KNOB = np.array([-0.207, 0.204])
q = r.move_pose([KNOB[0], KNOB[1], 1.05], down_quat(0.0), seconds=4.0, at_tcp=True)
print("q now", np.round(r.arm_q(), 4))
EOF
timeout 900 python3 -u step1.py 2>&1 | tail -6

# openrua op 23
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import cv2; im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('crop_eih.png', cv2.resize(im[120:480,160:480],None,fx=1.5,fy=1.5))"

# openrua op 24
cat > step2.py <<'EOF'
import numpy as np
from rob import *
r = Robot()
KNOB = np.array([-0.207, 0.204])
r.move_pose([KNOB[0], KNOB[1], 0.98], down_quat(0.0), seconds=2.5, at_tcp=True)
r.move_pose([KNOB[0], KNOB[1], 0.935], down_quat(0.0), seconds=2.0, at_tcp=True)
print("q now", np.round(r.arm_q(), 4))
w = r.wrench
print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
EOF
timeout 900 python3 -u step2.py 2>&1 | tail -6 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
cat > step3.py <<'EOF'
import sys, numpy as np
from rob import *
r = Robot()
delta = float(sys.argv[1])
gap = r.gripper(0.0)
q = r.arm_q()
q2 = list(q); q2[6] = q[6] + delta
print("j7", q[6], "->", q2[6])
r.move_q(q2, seconds=3.0)
print("gap after rotate", r.finger_gap())
r.gripper(0.04)
p, _ = r.tcp_pose(); print("tcp", np.round(p, 4))
EOF
timeout 900 python3 -u step3.py -1.5708 2>&1 | tail -6 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 26
cat > step4.py <<'EOF'
import numpy as np
from rob import *
r = Robot()
print("q", np.round(r.arm_q(), 4))
p, qt = r.tcp_pose()
r.move_pose([p[0], p[1], 1.08], qt, seconds=2.5, at_tcp=True)
EOF
timeout 900 python3 -u step4.py 2>&1 | tail -4 && timeout 120 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
cat > step5.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
r.move_pose([H[0], H[1], 1.15], down_quat(math.pi/2), seconds=4.0, at_tcp=True)
print("q", np.round(r.arm_q(), 4))
EOF
timeout 900 python3 -u step5.py 2>&1 | tail -4 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 28
timeout 300 python3 -u -c "
import numpy as np, math
from rob import *
r = Robot()
q = r.arm_q(); print('q', np.round(q,4))
p, qt = r.tcp_pose(); print('tcp', np.round(p,4), 'quat', np.round(qt,4), 'want', np.round(down_quat(math.pi/2),4))
R = quat_to_R(qt); print('hand x axis', np.round(R[:,0],3), 'hand z', np.round(R[:,2],3))
" 2>&1 | tail -3

# openrua op 29
timeout 900 python3 -u step5.py 2>&1 | tail -4

# openrua op 30
timeout 900 python3 -u step5.py 2>&1 | tail -4; timeout 300 python3 -u -c "
import numpy as np, math
from rob import *
r = Robot()
p, qt = r.tcp_pose(); print('tcp', np.round(p,4), 'quat', np.round(qt,4), 'want', np.round(down_quat(math.pi/2),4))
" 2>&1 | tail -1

# openrua op 31
python3 - <<'EOF'
s = open('/workspace/rob.py').read()
s = s.replace("""    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()""", """    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.01):
        for attempt in range(retries):
            code, err = self._move_q_once(q, seconds, via if attempt == 0 else None)
            if err <= tol:
                break
            print(f"move_q: retry {attempt + 1} (err {err:.4f})", flush=True)
        return code, err

    def _move_q_once(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()""")
open('/workspace/rob.py','w').write(s)
EOF
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 32
cat > step6.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
Q = down_quat(math.pi/2)
r.move_pose([H[0], H[1], 1.06], Q, seconds=2.5, at_tcp=True)
r.move_pose([H[0], H[1], 1.003], Q, seconds=2.0, at_tcp=True)
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
gap = r.gripper(0.0)
print("finger gap after close:", gap)
EOF
timeout 900 python3 -u step6.py 2>&1 | tail -6 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 33
cat > step7.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
Q = down_quat(math.pi/2)
r.move_pose([H[0], H[1], 1.10], Q, seconds=2.5, at_tcp=True)
print("gap", r.finger_gap())
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
EOF
timeout 900 python3 -u step7.py 2>&1 | tail -4 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 34
timeout 200 python3 -u cloud.py agentview >/dev/null; timeout 200 python3 -u cloud.py sideview >/dev/null; python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    pw = np.load(cam+'_cloud.npy').reshape(-1,3)
    m=(pw[:,0]>-0.10)&(pw[:,0]<-0.05)&(pw[:,1]>-0.11)&(pw[:,1]<0.04)&(pw[:,2]>0.905)&(pw[:,2]<1.1)
    h=pw[m]; print(cam,'handle n',len(h))
    for y0 in np.arange(-0.11, 0.04, 0.02):
        s = h[(h[:,1]>=y0)&(h[:,1]<y0+0.02)]
        if len(s): print('  y %.2f..%.2f n=%d x[%.3f %.3f] z[%.3f %.3f]'%(y0,y0+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,2].min(),s[:,2].max()))
"


# openrua op 35
timeout 200 python3 -u cloud.py birdview >/dev/null; python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
def sel(x0,x1,y0,y1,z0=0.905,z1=1.1):
    m=(pw[:,0]>x0)&(pw[:,0]<x1)&(pw[:,1]>y0)&(pw[:,1]<y1)&(pw[:,2]>z0)&(pw[:,2]<z1); return pw[m]
body = sel(-0.25,0.0,-0.35,-0.12)
print('body n',len(body),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(body[:,0].min(),body[:,0].max(),body[:,1].min(),body[:,1].max(),body[:,2].min(),body[:,2].max()))
print('body z hist', np.histogram(body[:,2], bins=np.arange(0.90,1.0,0.005)))
rim = body[body[:,2]>0.94]; print('rim center', np.round(rim.mean(0),3), 'x[%.3f %.3f] y[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
handle = sel(-0.15,-0.02,-0.12,0.06)
print('handle n',len(handle),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(handle[:,0].min(),handle[:,0].max(),handle[:,1].min(),handle[:,1].max(),handle[:,2].min(),handle[:,2].max()))
for y0 in np.arange(-0.12, 0.06, 0.02):
    s = handle[(handle[:,1]>=y0)&(handle[:,1]<y0+0.02)]
    if len(s): print(' y %.2f..%.2f n=%d x[%.3f %.3f] xmean %.3f z[%.3f %.3f] zmed %.3f'%(y0,y0+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,0].mean(),s[:,2].min(),s[:,2].max(),np.median(s[:,2])))
# knob and stove and moka now
k = sel(-0.30,-0.166,0.10,0.30); print('knob z[%.3f %.3f]'%(k[:,2].min(),k[:,2].max()))
s = sel(-0.164,0.044,0.11,0.297); print('stove z[%.3f %.3f]'%(s[:,2].min(),s[:,2].max()))
mk = sel(-0.01,0.09,-0.09,0.06); print('moka z[%.3f %.3f] x[%.3f %.3f] y[%.3f %.3f]'%(mk[:,2].min(),mk[:,2].max(),mk[:,0].min(),mk[:,0].max(),mk[:,1].min(),mk[:,1].max()))
"


# openrua op 36
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
def sel(x0,x1,y0,y1,z0=0.903,z1=1.1):
    m=(pw[:,0]>x0)&(pw[:,0]<x1)&(pw[:,1]>y0)&(pw[:,1]<y1)&(pw[:,2]>z0)&(pw[:,2]<z1); return pw[m]
body = sel(-0.25,0.0,-0.35,-0.12)
rim = body[body[:,2]>0.933]; print('rim n',len(rim),'center', np.round(rim.mean(0),3), 'x[%.3f %.3f] y[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
handle = sel(-0.15,-0.02,-0.125,0.06)
print('handle n',len(handle),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(handle[:,0].min(),handle[:,0].max(),handle[:,1].min(),handle[:,1].max(),handle[:,2].min(),handle[:,2].max()))
for y0 in np.arange(-0.12, 0.06, 0.02):
    s = handle[(handle[:,1]>=y0)&(handle[:,1]<y0+0.02)]
    if len(s): print(' y %.2f..%.2f n=%d x[%.3f %.3f] xmean %.3f z[%.3f %.3f] zmed %.3f'%(y0,y0+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,0].mean(),s[:,2].min(),s[:,2].max(),np.median(s[:,2])))
k = sel(-0.30,-0.166,0.10,0.30); print('knob z[%.3f %.3f]'%(k[:,2].min(),k[:,2].max()))
s = sel(-0.164,0.044,0.11,0.297); print('stove z[%.3f %.3f]'%(s[:,2].min(),s[:,2].max()))
mk = sel(-0.01,0.09,-0.09,0.06); print('moka z[%.3f %.3f] x[%.3f %.3f] y[%.3f %.3f]'%(mk[:,2].min(),mk[:,2].max(),mk[:,0].min(),mk[:,0].max(),mk[:,1].min(),mk[:,1].max()))
"


# openrua op 37
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node('urdf_get')
got=[]
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(String, '/robot_description', lambda m: got.append(m.data), qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n, timeout_sec=0.5)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
"; grep -n -A6 -E "joint name=\"panda_finger_joint1\"|joint name=\"panda_hand_joint\"|joint name=\"panda_hand_tcp_joint\"|link name=\"panda_leftfinger\"" robot.urdf | head -60

# openrua op 38
python3 -c "
import numpy as np, struct
def load_stl(fn):
    data=open(fn,'rb').read()
    if data[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)]); return v
    n=struct.unpack('<I',data[80:84])[0]
    arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f'),('v','<9f'),('a','<H')]))
    return arr['v'].reshape(-1,3)
for f in ['hand','finger']:
    v=load_stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',np.round(v.min(0),4),'max',np.round(v.max(0),4))
"

# openrua op 39
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand*.obj" -o -iname "finger*.obj" 2>/dev/null | grep -v proc | head

# openrua op 40
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,1]>-0.12)&(pw[:,1]<0.02)&(pw[:,0]>-0.35)&(pw[:,0]<0.2)&(pw[:,2]>1.12)&(pw[:,2]<1.30)
h=pw[m]; print('hand-ish n',len(h),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].min(),h[:,2].max()))
for z0 in np.arange(1.12,1.30,0.02):
    s=h[(h[:,2]>=z0)&(h[:,2]<z0+0.02)]
    if len(s): print(' z %.2f n=%d x[%.3f %.3f] y[%.3f %.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
m=(pw[:,1]>-0.12)&(pw[:,1]<0.02)&(pw[:,0]>-0.35)&(pw[:,0]<0.2)&(pw[:,2]>1.09)&(pw[:,2]<1.12)
f=pw[m]; print('fingers n',len(f), 'x[%.3f %.3f]'%(f[:,0].min(),f[:,0].max()) if len(f) else '')
"


# openrua op 41
timeout 600 python3 -u -c "
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
for name, p in [('pick_above',[-0.003,-0.22,1.05]),('pick',[-0.003,-0.22,0.915]),('carry',[-0.003,-0.22,1.20]),('place_above',[0.045,0.204,1.20]),('place',[0.045,0.204,0.95])]:
    s = r.ik(p, Q, at_tcp=True)
    print(name, p, None if s is None else np.round(s,3))
" 2>&1 | tail -5

# openrua op 42
cat > step8.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.gripper(0.04)
r.move_pose([-0.003, -0.22, 1.05], Q, seconds=4.0, at_tcp=True)
EOF
timeout 900 python3 -u step8.py 2>&1 | tail -4 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 43
timeout 200 python3 -u cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
pw = np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.25)&(pw[:,0]<0.1)&(pw[:,1]>-0.35)&(pw[:,1]<-0.12)&(pw[:,2]>0.925)&(pw[:,2]<0.95)
rim=pw[m]; print('rim n',len(rim),'x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),rim[:,2].min(),rim[:,2].max()))
for y0 in np.arange(-0.26,-0.18,0.01):
    s=rim[(rim[:,1]>=y0)&(rim[:,1]<y0+0.01)]
    if len(s): print(' y %.2f xmax %.3f xmin %.3f'%(y0, s[:,0].max(), s[:,0].min()))
t=pw[(pw[:,2]>0.89)&(pw[:,2]<0.91)]; print('table z med', np.median(t[:,2]), 'n', len(t))
"


# openrua op 44
timeout 200 python3 -u cloud.py birdview && python3 -c "
import numpy as np, cv2
pw = np.load('birdview_cloud.npy'); img = cv2.imread('birdview.png')
# dark pixels in the pan region (rows 230-330, cols 200-300)
sub = img[230:330, 200:300]; dark = (sub.max(axis=2) < 60)
ys, xs = np.where(dark); print('dark px rows', ys.min()+230, ys.max()+230, 'cols', xs.min()+200, xs.max()+200)
# world coords of extreme dark pixels
for (rr,cc) in [(ys.min()+230, int(np.median(xs[ys==ys.min()]))+200),(ys.max()+230,int(np.median(xs[ys==ys.max()]))+200),(int(np.median(ys[xs==xs.min()]))+230,xs.min()+200),(int(np.median(ys[xs==xs.max()]))+230,xs.max()+200)]:
    print((rr,cc), np.round(pw[rr,cc],3))
m = (pw[...,2]>0.925)&(pw[...,2]<0.95)
sub=pw[220:340,180:320]; zz=sub[...,2]; mm=(zz>0.925)&(zz<0.95); rim=sub[mm]
print('rim by depth x[%.3f %.3f] y[%.3f %.3f]'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max()))
"


# openrua op 45
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.3)&(pw[:,0]<0.15)&(pw[:,1]>-0.35)&(pw[:,1]<-0.10)&(pw[:,2]>0.903)&(pw[:,2]<0.96)
p=pw[m]
for y0 in np.arange(-0.33,-0.10,0.02):
    s=p[(p[:,1]>=y0)&(p[:,1]<y0+0.02)]
    if len(s): print(' y %.2f n=%d x[%.3f %.3f] zmax %.3f'%(y0,len(s),s[:,0].min(),s[:,0].max(),s[:,2].max()))
rim=p[p[:,2]>0.93]; print('rim center', np.round(rim.mean(0),3))
e = np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
m=(e[:,0]>-0.3)&(e[:,0]<0.15)&(e[:,1]>-0.35)&(e[:,1]<-0.10)&(e[:,2]>0.925)&(e[:,2]<0.96)
rim=e[m]; print('eih rim center', np.round(rim.mean(0),3), 'y[%.3f %.3f]'%(rim[:,1].min(),rim[:,1].max()))
"


# openrua op 46
python3 -c "
import numpy as np
e = np.load('robot0_eye_in_hand_cloud.npy').reshape(-1,3)
m=(e[:,0]>-0.05)&(e[:,0]<0.1)&(e[:,1]>-0.25)&(e[:,1]<-0.21)&(e[:,2]>0.90)&(e[:,2]<0.96)
p=e[m]
for z0 in np.arange(0.90,0.945,0.005):
    s=p[(p[:,2]>=z0)&(p[:,2]<z0+0.005)]
    if len(s): print(' z %.3f n=%d x[%.3f %.3f]'%(z0,len(s),s[:,0].min(),s[:,0].max()))
"


# openrua op 47
cat > step9.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
G = [0.012, -0.23]
r.move_pose([G[0], G[1], 0.98], Q, seconds=2.5, at_tcp=True)
r.move_pose([G[0], G[1], 0.91], Q, seconds=2.0, at_tcp=True)
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
EOF
timeout 900 python3 -u step9.py 2>&1 | tail -5 && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 48
cat > step10.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
gap = r.gripper(0.0)
r.move_pose([0.012, -0.23, 0.99], Q, seconds=2.5, at_tcp=True)
print("gap after lift", r.finger_gap())
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
EOF
timeout 900 python3 -u step10.py 2>&1 | tail -5 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 49
timeout 120 python3 tools/perception/cam_snap.py frontview

# openrua op 50
cat > step11.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.move_pose([0.012, -0.23, 1.12], Q, seconds=2.5, at_tcp=True)
print("gap", r.finger_gap())
EOF
timeout 900 python3 -u step11.py 2>&1 | tail -3 && timeout 200 python3 -u cloud.py birdview && python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.3)&(pw[:,0]<0.1)&(pw[:,1]>-0.4)&(pw[:,1]<-0.10)&(pw[:,2]>0.903)&(pw[:,2]<1.3)
p=pw[m]
print(np.histogram(p[:,2], bins=np.arange(0.90,1.32,0.02)))
"

# openrua op 51
cat > step12.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
r.move_pose([0.012, -0.23, 1.25], Q, seconds=2.5, at_tcp=True); print("gap", r.finger_gap())
r.move_pose([0.045, 0.204, 1.25], Q, seconds=5.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
r.move_pose([0.045, 0.204, 1.02], Q, seconds=3.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
EOF
timeout 1500 python3 -u step12.py 2>&1 | tail -8 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 52
cat > step13.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
for z in [0.995, 0.975]:
    r.move_pose([0.045, 0.204, z], Q, seconds=2.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
r.gripper(0.04)
r.move_pose([0.045, 0.204, 1.10], Q, seconds=2.5, at_tcp=True)
EOF
timeout 1500 python3 -u step13.py 2>&1 | tail -8 && timeout 120 python3 tools/perception/cam_snap.py agentview

# openrua op 53
cat > step14.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.move_pose([0.045, 0.204, 1.25], Q, seconds=2.5, at_tcp=True)
home = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]
r.move_q(home, seconds=5.0)
print("q", np.round(r.arm_q(), 3))
EOF
timeout 1500 python3 -u step14.py 2>&1 | tail -4 && for c in agentview birdview sideview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c; done

# openrua op 54
timeout 200 python3 -u cloud.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
pw = np.load('birdview_cloud.npy').reshape(-1,3)
def sel(x0,x1,y0,y1,z0,z1):
    m=(pw[:,0]>x0)&(pw[:,0]<x1)&(pw[:,1]>y0)&(pw[:,1]<y1)&(pw[:,2]>z0)&(pw[:,2]<z1); return pw[m]
# pan rim: points above the stove top (0.93) in the stove area + around
p = sel(-0.30,0.20,0.05,0.45,0.94,1.1)
print('pan pts n',len(p),'z[%.3f %.3f]'%(p[:,2].min(),p[:,2].max()))
print(np.histogram(p[:,2], bins=np.arange(0.94,1.02,0.01)))
rim = p[(p[:,2]>0.96)&(p[:,0]<0.08)&(p[:,1]<0.32)]   # exclude handle (y>0.3) 
print('rim x[%.3f %.3f] y[%.3f %.3f] -> center (%.3f, %.3f)'%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),(rim[:,0].min()+rim[:,0].max())/2,(rim[:,1].min()+rim[:,1].max())/2))
handle = p[(p[:,1]>0.30)]
print('handle x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]'%(handle[:,0].min(),handle[:,0].max(),handle[:,1].min(),handle[:,1].max(),handle[:,2].min(),handle[:,2].max()))
# stove plate (visible part) center check
s = sel(-0.30,0.20,0.05,0.45,0.912,0.935)
print('stove visible x[%.3f %.3f] y[%.3f %.3f]'%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
# knob fin
k = sel(-0.30,-0.166,0.10,0.30,0.94,0.97)
print('fin n',len(k),'x[%.3f %.3f] y[%.3f %.3f]'%(k[:,0].min(),k[:,0].max(),k[:,1].min(),k[:,1].max()))
c = k.mean(0); u,s_,vt = np.linalg.svd(k[:,:2]-c[:2]); print('fin axis', np.round(vt[0],3), 'angle deg', np.degrees(np.arctan2(vt[0][1],vt[0][0])))
"


# openrua op 55
python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.30)&(pw[:,0]<0.20)&(pw[:,1]>0.0)&(pw[:,1]<0.46)&(pw[:,2]>0.935)&(pw[:,2]<1.1)
p=pw[m]
for y0 in np.arange(0.02,0.46,0.02):
    s=p[(p[:,1]>=y0)&(p[:,1]<y0+0.02)]
    if len(s): print(' y %.2f n=%3d x[%.3f %.3f] z[%.3f %.3f]'%(y0,len(s),s[:,0].min(),s[:,0].max(),s[:,2].min(),s[:,2].max()))
"


# openrua op 56
python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[200:330,300:460],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 57
cat > step15.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
r.gripper(0.0)
r.move_pose([0.080, 0.20, 1.10], Q, seconds=5.0, at_tcp=True)
r.move_pose([0.080, 0.20, 0.955], Q, seconds=3.0, at_tcp=True); print("wrench", wr())
r.move_pose([0.052, 0.20, 0.955], Q, seconds=3.0, at_tcp=True); print("wrench", wr())
r.move_pose([0.052, 0.20, 1.10], Q, seconds=2.5, at_tcp=True)
EOF
timeout 1500 python3 -u step15.py 2>&1 | grep -E "move_pose|wrench|gripper" && timeout 200 python3 -u cloud.py birdview >/dev/null && python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.16)&(pw[:,0]<0.20)&(pw[:,1]>0.08)&(pw[:,1]<0.32)&(pw[:,2]>0.96)&(pw[:,2]<0.98)
p=pw[m]; print('rim x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f] center (%.3f, %.3f)'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),(p[:,0].min()+p[:,0].max())/2,(p[:,1].min()+p[:,1].max())/2))
"

# openrua op 58
cat > step16.py <<'EOF'
import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.move_pose([0.052, 0.20, 1.25], Q, seconds=2.5, at_tcp=True)
home = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]
r.move_q(home, seconds=5.0)
r.gripper(0.04)
EOF
timeout 1500 python3 -u step16.py 2>&1 | grep -E "move_pose|gripper|error" ; timeout 200 python3 -u cloud.py birdview >/dev/null && python3 -c "
import numpy as np
pw = np.load('birdview_cloud.npy').reshape(-1,3)
m=(pw[:,0]>-0.16)&(pw[:,0]<0.20)&(pw[:,1]>0.08)&(pw[:,1]<0.31)&(pw[:,2]>0.96)&(pw[:,2]<0.98)
p=pw[m]; print('rim x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f] center (%.3f, %.3f)'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max(),(p[:,0].min()+p[:,0].max())/2,(p[:,1].min()+p[:,1].max())/2))
for y0 in np.arange(0.08,0.32,0.02):
    s=p[(p[:,1]>=y0)&(p[:,1]<y0+0.02)]
    if len(s): print(' y %.2f n=%3d x[%.3f %.3f] z[%.3f %.3f]'%(y0,len(s),s[:,0].min(),s[:,0].max(),s[:,2].min(),s[:,2].max()))
s=pw[(pw[:,0]>-0.30)&(pw[:,0]<0.20)&(pw[:,1]>0.05)&(pw[:,1]<0.45)&(pw[:,2]>0.912)&(pw[:,2]<0.935)]
print('stove visible x[%.3f %.3f] y[%.3f %.3f]'%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
k=pw[(pw[:,0]>-0.30)&(pw[:,0]<-0.166)&(pw[:,1]>0.10)&(pw[:,1]<0.30)&(pw[:,2]>0.94)&(pw[:,2]<0.97)]
c=k.mean(0); u,s_,vt=np.linalg.svd(k[:,:2]-c[:2]); print('fin angle deg', np.degrees(np.arctan2(vt[0][1],vt[0][0])), 'n', len(k))
" && for c in agentview birdview frontview; do timeout 120 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo snaps done

# openrua op 59
python3 -c "
import cv2; im=cv2.imread('frontview.png'); cv2.imwrite('crop_front.png', cv2.resize(im[300:420,340:520],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 60
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-ik-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-ik-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-ik-quirks
description: Verified quirks of the /workspace Panda (LIBERO-style kitchen) robot: IK frame, tip link, settling objects, joint7 lag
metadata:
  type: project
---

Verified 2026-09-12 on the Franka Panda onboard computer (/workspace, machine.yaml):
- `/compute_fk` returns poses already in WORLD (panda_link0 at world (-0.66, 0, 0.912)).
- `/compute_ik` with empty frame_id interprets poses in the same WORLD frame (a base-frame pose failed outright), despite docs/30-action.md saying "arm base".
- IK tip link is `panda_link8`, not `panda_hand`: for a desired hand quaternion multiply by Rz(+45°) before sending (hand = link8 rotated -45°).
- TCP offset 0.1034 is the fingertip END; pads cover ~2 cm above it.
- At session start objects can float several cm above supports and only settle after the first commanded motion; re-snapshot before planning grasps.
- Large joint7 moves often end with error_code -5 and a big residual; simply resend the same goal (a retry loop in a move helper fixes it).
- Stove knob: vertical-axis disc with a fin; ~30° rotation (grasp fin, rotate j7) turns the burner red (= on). Franka hand is ~20 cm wide along the finger axis — mind neighbours like the moka pot.

**Why:** cost several turns to discover; docs are generic and partly wrong for this machine.
**How to apply:** reuse /workspace/rob.py (Robot class) conventions if it still exists; otherwise apply the above when writing IK/FK helpers.
OPENRUA_EOF

# openrua op 61
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 62
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda workstation IK quirks](panda-workstation-ik-quirks.md) — FK/IK are world-frame, tip link is panda_link8, objects settle late, retry j7 moves
OPENRUA_EOF
