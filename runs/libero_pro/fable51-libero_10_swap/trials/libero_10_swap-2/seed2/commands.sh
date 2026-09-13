#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 2
cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py; echo ----; cat /workspace/tools/action/gripper_cmd.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | head -50; echo ----; ros2 node list

# openrua op 4
ros2 service list | grep -v -E "parameter|get_type_description" ; echo ----; ros2 topic echo /pipeline_state --once 2>&1 | head -30; echo ----; ros2 topic echo /joint_states --once

# openrua op 5
cat /tmp/claude-1000/-workspace/6e9309cb-97d7-468b-85d2-0551193d22cb/tasks/bn00e4fqv.output

# openrua op 6
ros2 topic info /pipeline_state -v 2>&1 | head -30; echo ----; timeout 30 ros2 topic echo /joint_states --once

# openrua op 7
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls -la *.png

# openrua op 8
for p in "240 285" "320 305" "376 283" "378 238" "383 312" "300 200"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12

# openrua op 10
mkdir -p "$(dirname /workspace/pc.py)"
cat > /workspace/pc.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's current depth frame as world-frame XYZ (H x W x 3) .npy
plus the color PNG.  Usage: python3 pc.py <camera>
Writes <camera>_xyz.npy and <camera>.png
"""
import sys
import numpy as np
import rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("pc_" + cam)
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 15
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + T
    xyz[~np.isfinite(depth)] = np.nan
    np.save(f"{cam}_xyz.npy", xyz)
    cv2.imwrite(f"{cam}.png", color)
    print(f"{cam}: {W}x{H} cam at {T.round(3)} saved {cam}_xyz.npy")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 11
for c in birdview frontview sideview agentview; do timeout 120 python3 pc.py $c & done; wait

# openrua op 12
python3 -c "
import numpy as np
np.set_printoptions(precision=3, suppress=True)
X=np.load('birdview_xyz.npy')
z=X[...,2]
# objects above table (z>0.905) within table region
m=(z>0.905)&(X[...,0]>-0.4)&(X[...,0]<0.4)&(np.abs(X[...,1])<0.5)
print('above-table pts', m.sum())
# cluster by simple connected components
import cv2
lab_n, lab = cv2.connectedComponents(m.astype(np.uint8))
for i in range(1,lab_n):
    s=lab==i
    if s.sum()<10: continue
    P=X[s]
    print(i, s.sum(), 'x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'zmax',P[:,2].max().round(3),'centroid',P.mean(0).round(3))
"


# openrua op 13
python3 -c "
import numpy as np
X=np.load('birdview_xyz.npy')
xs=np.arange(-0.16,0.16,0.01); ys=np.arange(-0.12,0.32,0.01)
z=X[...,2]
print('rows=x (top=-0.16), cols=y (left=-0.12) ; value = max z above table in cm (0=table)')
print('     '+' '.join(f'{int(round(y*100)):3d}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(np.abs(X[...,0]-x)<0.005)&(np.abs(X[...,1]-y)<0.005)
        row.append(int(round((np.nanmax(z[m])-0.90)*100)) if m.any() else -1)
    print(f'{int(round(x*100)):4d} '+' '.join(f'{v:3d}' for v in row))
"


# openrua op 14
python3 -c "
import numpy as np
X=np.load('birdview_xyz.npy')
xs=np.arange(-0.10,0.0,0.005); ys=np.arange(-0.35,-0.19,0.005)
z=X[...,2]
print('rows=x, cols=y (units 0.5cm); value = max z above table in cm')
print('      '+' '.join(f'{y*100:5.1f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(np.abs(X[...,0]-x)<0.0025)&(np.abs(X[...,1]-y)<0.0025)
        row.append(int(round((np.nanmax(z[m])-0.90)*100)) if m.any() else -1)
    print(f'{x*100:5.1f} '+' '.join(f'{v:5d}' for v in row))
"


# openrua op 15
python3 -c "
import numpy as np
for cam in ['frontview','sideview','agentview']:
    X=np.load(cam+'_xyz.npy')
    m=(X[...,0]>-0.12)&(X[...,0]<0.02)&(X[...,1]>-0.36)&(X[...,1]<-0.18)&(X[...,2]>0.902)
    P=X[m]
    print(cam, 'pts', len(P))
    for h in np.arange(0.0,0.17,0.01):
        s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.01)
        if s.sum()<3: print(f'  h={h*100:4.0f}cm: -'); continue
        Q=P[s]
        print(f'  h={h*100:4.0f}cm n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"


# openrua op 16
python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[290:400,130:240]; cv2.imwrite('crop_front_moka.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); c=im[200:340,90:220]; cv2.imwrite('crop_agent_moka.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 17
python3 -c "
import cv2
im=cv2.imread('robot0_eye_in_hand.png'); c=im[200:330,80:210]; cv2.imwrite('crop_eih_knob.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('robot0_robotview.png'); c=im[90:190,430:560]; cv2.imwrite('crop_rv_knob.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); c=im[300:380,330:420]; cv2.imwrite('crop_sv_knob.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 18
timeout 30 ros2 node info /robot_bridge 2>&1 | grep -v -E "parameter|type_description"

# openrua op 19
cd /workspace/.ros/log && wc -l * && tail -n 20 $(ls -S | head -1)

# openrua op 20
python3 -c "
import numpy as np
for cam in ['frontview','sideview','birdview']:
    X=np.load(cam+'_xyz.npy')
    m=(X[...,0]>-0.27)&(X[...,0]<-0.13)&(X[...,1]>0.13)&(X[...,1]<0.27)&(X[...,2]>0.902)
    P=X[m]
    print(cam, 'pts', len(P))
    for h in np.arange(0.0,0.07,0.005):
        s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.005)
        if s.sum()<3: print(f'  h={h*100:4.1f}cm: -'); continue
        Q=P[s]
        print(f'  h={h*100:4.1f}cm n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"


# openrua op 21
cd /workspace && python3 -c "
import numpy as np
for cam in ['frontview','sideview','birdview']:
    X=np.load(cam+'_xyz.npy')
    m=(X[...,0]>-0.27)&(X[...,0]<-0.13)&(X[...,1]>0.13)&(X[...,1]<0.27)&(X[...,2]>0.902)
    P=X[m]
    print(cam, 'pts', len(P))
    for h in np.arange(0.0,0.07,0.005):
        s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.005)
        if s.sum()<3: print(f'  h={h*100:4.1f}cm: -'); continue
        Q=P[s]
        print(f'  h={h*100:4.1f}cm n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"


# openrua op 22
mkdir -p "$(dirname /workspace/ctl_server.py)"
cat > /workspace/ctl_server.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent controller: builds ROS clients once, serves JSON commands over
a local TCP socket (port 5555).  Run: python3 -u ctl_server.py > ctl.log 2>&1 &
Client: python3 c.py '{"cmd": "js"}'
"""
import json
import socket
import threading
import traceback
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(Path("machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# world = base + BASE_IN_WORLD (from tf world->panda_link0)
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl_server")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(30); self.grip.wait_for_server(30)
        self.ik.wait_for_service(30); self.fk.wait_for_service(30)
        self.spin(1.0)

    def spin(self, t):
        end = self.node.get_clock().now().nanoseconds / 1e9 + t
        # sim clock may be paused: also bound by wall iterations
        for _ in range(int(t / 0.05) + 1):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _js(self, m):
        self.js = m

    # ---------------- sensing ----------------
    def cmd_js(self):
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.js is not None:
                break
        d = dict(zip(self.js.name, self.js.position))
        return {"arm": [d[j] for j in ARM],
                "fingers": [d.get("panda_finger_joint1"), d.get("panda_finger_joint2")]}

    def arm_seed(self):
        js = self.cmd_js()
        s = JointState(); s.name = list(ARM); s.position = list(js["arm"])
        return s

    def cmd_fk(self, joints=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_seed() if joints is None else self._seed(joints)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return {"error": "fk failed", "code": None if r is None else r.error_code.val}
        p = r.pose_stamped[0].pose
        pos_b = np.array([p.position.x, p.position.y, p.position.z])
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        pos_w = pos_b + BASE_IN_WORLD
        tcp_w = pos_w + TCP * quat_to_R(q)[:, 2]
        return {"hand_world": pos_w.round(4).tolist(), "tcp_world": tcp_w.round(4).tolist(),
                "quat": [round(v, 4) for v in q]}

    def _seed(self, joints):
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in joints]
        return s

    # ---------------- IK ----------------
    def cmd_ik(self, pos, quat, at="tcp", seed=None):
        pos = np.array(pos, float)
        if at == "tcp":
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pb = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self.arm_seed() if seed is None else self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            return {"error": "ik timeout"}
        if r.error_code.val != 1:
            return {"error": "ik failed", "code": r.error_code.val}
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return {"joints": [sol[j] for j in ARM]}

    # ---------------- motion ----------------
    def cmd_joints(self, joints, t=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                tt = t * (i + 1) / n
                pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in joints])
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            return {"error": "goal not accepted"}
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        if res.result() is None:
            return {"error": "result timeout"}
        code = res.result().result.error_code
        self.spin(0.3)
        js = self.cmd_js()
        err = float(np.max(np.abs(np.array(js["arm"]) - np.array(joints, float))))
        return {"error_code": code, "max_joint_err": round(err, 4), "arm": [round(v, 4) for v in js["arm"]]}

    def cmd_move(self, pos, quat, t=3.0, at="tcp", seed=None):
        ik = self.cmd_ik(pos, quat, at, seed)
        if "error" in ik:
            return ik
        r = self.cmd_joints(ik["joints"], t)
        r["fk"] = self.cmd_fk()
        r["ik_joints"] = [round(v, 4) for v in ik["joints"]]
        return r

    def cmd_grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(0.3)
        return {"reached": r.reached_goal, "stalled": r.stalled, "position": r.position,
                "fingers": self.cmd_js()["fingers"]}

    def cmd_servo(self, lin, ang=(0, 0, 0), n=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(int(n)):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.spin(0.3)
        return {"fk": self.cmd_fk()}

    def cmd_snap(self, cam, out=None):
        got = {}
        topic = f"/{cam}/color/image_raw"
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        for _ in range(600):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if "m" in got:
                break
        self.node.destroy_subscription(sub)
        if "m" not in got:
            return {"error": "no image"}
        out = out or f"{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got["m"], "bgr8"))
        return {"saved": out}

    def handle(self, d):
        cmd = d.pop("cmd")
        return getattr(self, "cmd_" + cmd)(**d)


def main():
    ctl = Ctl()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 5555)); srv.listen(1)
    print("ready", flush=True)
    while True:
        conn, _ = srv.accept()
        data = b""
        while not data.endswith(b"\n"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        try:
            d = json.loads(data.decode())
            print(">>", d, flush=True)
            out = ctl.handle(d)
        except Exception as e:  # noqa
            out = {"error": repr(e), "tb": traceback.format_exc()}
        print("<<", out, flush=True)
        conn.sendall((json.dumps(out) + "\n").encode())
        conn.close()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 23
mkdir -p "$(dirname /workspace/c.py)"
cat > /workspace/c.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Client for ctl_server: python3 c.py '{"cmd": "js"}'"""
import json
import socket
import sys

s = socket.create_connection(("127.0.0.1", 5555))
s.sendall((sys.argv[1].strip() + "\n").encode())
data = b""
while not data.endswith(b"\n"):
    chunk = s.recv(65536)
    if not chunk:
        break
    data += chunk
out = json.loads(data.decode())
tb = out.pop("tb", None)
print(json.dumps(out, indent=1))
if tb:
    print(tb)
OPENRUA_EOF

# openrua op 24
(nohup python3 -u ctl_server.py > ctl.log 2>&1 &) ; sleep 15; cat ctl.log

# openrua op 25
timeout 120 python3 c.py '{"cmd":"js"}'; timeout 120 python3 c.py '{"cmd":"fk"}'

# openrua op 26
python3 - <<'EOF'
import json, subprocess
def c(d): 
    return json.loads(subprocess.run(["python3","c.py",json.dumps(d)],capture_output=True,text=True).stdout)
# Test IK in both frame interpretations for the current hand pose
q=[0.9996,0,-0.0284,0.0]
# world interpretation (server subtracts BASE_IN_WORLD internally => sends base coords)
print("world-interp:", c({"cmd":"ik","pos":[-0.203,0,1.27],"quat":q,"at":"hand"}))
# pretend base coords are world (server will subtract offset again -> wrong if IK is base-frame)
print("base-as-world:", c({"cmd":"ik","pos":[-0.203-0.66,0,1.27+0.912],"quat":q,"at":"hand"}))
EOF

# openrua op 27
timeout 120 python3 c.py '{"cmd":"fk","joints":[0.058461300098655994, -0.16195109733096721, -0.057909481834373125, -2.444193585478249, -0.011654179947540132, 2.225666621365345, 0.00893396426828436]}'

# openrua op 28
python3 - <<'EOF'
import re
s=open('ctl_server.py').read()
s=s.replace('''# world = base + BASE_IN_WORLD (from tf world->panda_link0)
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])''','''# measured: /compute_fk and /compute_ik on this machine work in the WORLD
# frame (they match tf world->panda_hand), so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
# IK's tip link is the flange (panda_link8), which is panda_hand rotated
# +45deg about z.  quat args everywhere are the desired panda_hand quat.
def quat_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz]
RZ45 = [0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8)]''')
s=s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''','''        q8 = quat_mul(list(quat), RZ45)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open('ctl_server.py','w').write(s)
EOF
pkill -f ctl_server.py; sleep 1; (nohup python3 -u ctl_server.py > ctl.log 2>&1 &); sleep 12; cat ctl.log
timeout 120 python3 c.py '{"cmd":"ik","pos":[-0.203,0,1.27],"quat":[0.9996,0,-0.0284,0.0],"at":"hand"}'

# openrua op 29
pgrep -af ctl_server; (nohup python3 -u ctl_server.py > ctl.log 2>&1 &); sleep 12; cat ctl.log

# openrua op 30
ps -o pid,lstart,cmd -p 1231; kill 1231; sleep 2; pgrep -af "ctl_server" | grep -v bash; (nohup python3 -u ctl_server.py > ctl.log 2>&1 &); sleep 12; cat ctl.log

# openrua op 31
kill -9 1231; sleep 2; pgrep -af "ctl_server" | grep -v bash; echo "---"; (nohup python3 -u ctl_server.py > ctl.log 2>&1 &); sleep 12; cat ctl.log

# openrua op 32
timeout 120 python3 c.py '{"cmd":"ik","pos":[-0.203,0,1.27],"quat":[0.9996,0,-0.0284,0.0],"at":"hand"}'; timeout 120 python3 c.py '{"cmd":"fk"}'

# openrua op 33
timeout 300 python3 c.py '{"cmd":"grip","width":0.04}'; timeout 600 python3 c.py '{"cmd":"move","pos":[-0.200,0.200,1.06],"quat":[1,0,0,0],"t":4}'

# openrua op 34
timeout 120 python3 c.py '{"cmd":"snap","cam":"robot0_eye_in_hand","out":"eih1.png"}'

# openrua op 35
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.200,0.200,0.948],"quat":[1,0,0,0],"t":3}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"robot0_eye_in_hand","out":"eih2.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"sideview","out":"side2.png"}'

# openrua op 36
python3 -c "
import cv2
im=cv2.imread('side2.png'); c=im[250:400,280:460]; cv2.imwrite('crop_side2.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 37
timeout 300 python3 c.py '{"cmd":"grip","width":0.0}'; timeout 60 python3 c.py '{"cmd":"js"}'

# openrua op 38
timeout 600 python3 c.py '{"cmd":"joints","joints":[0.22276700542243857,0.4530502674948867,0.16015380983956798,-2.3600757584034864,-0.204829897799911,2.8011664750346705,0.55],"t":3}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"robot0_eye_in_hand","out":"eih3.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"agentview","out":"agent3.png"}'

# openrua op 39
timeout 600 python3 c.py '{"cmd":"joints","joints":[0.2222,0.4518,0.1579,-2.3617,-0.2034,2.8004,0.8168],"t":1}'; timeout 300 python3 c.py '{"cmd":"grip","width":0.04}'; timeout 600 python3 c.py '{"cmd":"move","pos":[-0.200,0.200,1.08],"quat":[1,0,0,0],"t":3}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"agentview","out":"agent4.png"}'

# openrua op 40
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.047,-0.260,1.14],"quat":[0.70710678,0.70710678,0,0],"t":4}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"robot0_eye_in_hand","out":"eih5.png"}'

# openrua op 41
timeout 600 python3 c.py '{"cmd":"joints","joints":[-0.3798,0.4514,-0.0245,-1.7038,0.0128,2.155,-1.1944],"t":4}'; timeout 120 python3 c.py '{"cmd":"fk"}'

# openrua op 42
timeout 600 python3 c.py '{"cmd":"joints","joints":[-0.3798,0.4514,-0.0245,-1.7038,0.0128,2.155,-1.1944],"t":7}'; timeout 120 python3 c.py '{"cmd":"fk"}'

# openrua op 43
timeout 120 python3 pc.py robot0_eye_in_hand && python3 -c "
import numpy as np
X=np.load('robot0_eye_in_hand_xyz.npy')
m=(X[...,2]>0.95)&(X[...,2]<1.1)&(np.abs(X[...,0]+0.047)<0.08)&(np.abs(X[...,1]+0.26)<0.1)
P=X[m]; print('pts',len(P))
for h in np.arange(0.05,0.17,0.01):
    s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.01)
    if s.sum()<3: continue
    Q=P[s]; print(f'h={h*100:3.0f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] cx={(Q[:,0].min()+Q[:,0].max())/2:.3f} cy={(Q[:,1].min()+Q[:,1].max())/2:.3f}')
"

# openrua op 44
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.046,-0.259,1.045],"quat":[0.70710678,0.70710678,0,0],"t":4}'

# openrua op 45
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.046,-0.259,0.972],"quat":[0.70710678,0.70710678,0,0],"t":4}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"frontview","out":"front6.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"robot0_eye_in_hand","out":"eih6.png"}'

# openrua op 46
python3 -c "
import cv2
im=cv2.imread('front6.png'); c=im[230:400,100:280]; cv2.imwrite('crop_front6.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 47
timeout 60 python3 c.py '{"cmd":"js"}'

# openrua op 48
timeout 120 python3 pc.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np
X=np.load('robot0_eye_in_hand_xyz.npy')
m=(X[...,2]>0.95)&(X[...,2]<1.1)&(np.abs(X[...,0]+0.047)<0.1)&(np.abs(X[...,1]+0.26)<0.12)
P=X[m]; print('pts',len(P))
for h in np.arange(0.05,0.17,0.01):
    s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.01)
    if s.sum()<3: continue
    Q=P[s]; print(f'h={h*100:3.0f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] cx={(Q[:,0].min()+Q[:,0].max())/2:.3f} cy={(Q[:,1].min()+Q[:,1].max())/2:.3f}')
# finger pads: look for points at z 1.02-1.06 far from pot center in x
"; timeout 60 python3 c.py '{"cmd":"fk"}'

# openrua op 49
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.046,-0.259,1.12],"quat":[0.70710678,0.70710678,0,0],"t":4}' | head -5; timeout 120 python3 pc.py birdview >/dev/null && python3 -c "
import numpy as np
X=np.load('birdview_xyz.npy')
m=(X[...,2]>0.905)&(X[...,2]<1.1)&(np.abs(X[...,0]+0.047)<0.08)&(np.abs(X[...,1]+0.26)<0.12)
P=X[m]; print('pts',len(P))
for h in np.arange(0.05,0.17,0.01):
    s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.01)
    if s.sum()<3: continue
    Q=P[s]; print(f'h={h*100:3.0f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] cx={(Q[:,0].min()+Q[:,0].max())/2:.3f}')
# handle: points with y < -0.30
H=P[P[:,1]<-0.30]; print('handle x',H[:,0].min().round(3),H[:,0].max().round(3),'y',H[:,1].min().round(3),H[:,1].max().round(3),'z',H[:,2].min().round(3),H[:,2].max().round(3))
"

# openrua op 50
timeout 120 python3 pc.py frontview >/dev/null; timeout 120 python3 pc.py sideview > /dev/null; python3 -c "
import numpy as np
for cam in ['frontview','sideview']:
    X=np.load(cam+'_xyz.npy')
    m=(X[...,2]>0.902)&(X[...,2]<1.08)&(np.abs(X[...,0]+0.047)<0.09)&(np.abs(X[...,1]+0.26)<0.12)
    P=X[m]; print(cam,'pts',len(P))
    for h in np.arange(0.0,0.17,0.01):
        s=(P[:,2]-0.90>=h)&(P[:,2]-0.90<h+0.01)
        if s.sum()<3: continue
        Q=P[s]; print(f'  h={h*100:3.0f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"

# openrua op 51
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,-0.334,1.12],"quat":[0.70710678,0.70710678,0,0],"t":3}' | grep -A4 tcp_world; timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,-0.334,1.00],"quat":[0.70710678,0.70710678,0,0],"t":4}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"frontview","out":"front7.png"}'

# openrua op 52
python3 -c "
import cv2
im=cv2.imread('front7.png'); c=im[220:400,80:280]; cv2.imwrite('crop_front7.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 53
timeout 300 python3 c.py '{"cmd":"grip","width":0.0}'; timeout 60 python3 c.py '{"cmd":"js"}' | tail -5

# openrua op 54
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,-0.334,1.15],"quat":[0.70710678,0.70710678,0,0],"t":4}' | grep -A4 -E "error_code|tcp_world"; timeout 60 python3 c.py '{"cmd":"js"}' | tail -4; timeout 120 python3 c.py '{"cmd":"snap","cam":"frontview","out":"front8.png"}'

# openrua op 55
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,-0.334,1.20],"quat":[0.70710678,0.70710678,0,0],"t":3}' | grep -A1 -E "error_code|max_joint"; timeout 900 python3 c.py '{"cmd":"move","pos":[-0.050,0.131,1.20],"quat":[0.70710678,0.70710678,0,0],"t":8}'; timeout 60 python3 c.py '{"cmd":"js"}' | tail -4

# openrua op 56
timeout 120 python3 c.py '{"cmd":"snap","cam":"agentview","out":"agent9.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"sideview","out":"side9.png"}'

# openrua op 57
timeout 120 python3 pc.py frontview >/dev/null; timeout 120 python3 pc.py sideview > /dev/null; timeout 120 python3 pc.py agentview > /dev/null; python3 -c "
import numpy as np
for cam in ['frontview','sideview','agentview']:
    X=np.load(cam+'_xyz.npy')
    m=(X[...,2]>1.04)&(X[...,2]<1.30)&(X[...,0]>-0.13)&(X[...,0]<0.03)&(X[...,1]>0.10)&(X[...,1]<0.30)
    P=X[m]; print(cam,'pts',len(P))
    for h in np.arange(1.04,1.30,0.01):
        s=(P[:,2]>=h)&(P[:,2]<h+0.01)
        if s.sum()<3: continue
        Q=P[s]; print(f'  z={h:.2f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]')
"

# openrua op 58
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,0.1425,1.05],"quat":[0.70710678,0.70710678,0,0],"t":5}' | grep -A4 -E "error_code|tcp_world"; timeout 120 python3 c.py '{"cmd":"snap","cam":"agentview","out":"agent10.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"sideview","out":"side10.png"}'

# openrua op 59
python3 -c "
import cv2
im=cv2.imread('agent10.png'); c=im[150:360,330:540]; cv2.imwrite('crop_agent10.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('side10.png'); c=im[200:400,200:450]; cv2.imwrite('crop_side10.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_NEAREST))"

# openrua op 60
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,0.1425,1.036],"quat":[0.70710678,0.70710678,0,0],"t":3}' | grep -A4 -E "error_code|tcp_world"; timeout 300 python3 c.py '{"cmd":"grip","width":0.04}'

# openrua op 61
timeout 600 python3 c.py '{"cmd":"move","pos":[-0.050,0.1425,1.16],"quat":[0.70710678,0.70710678,0,0],"t":4}' | grep -A1 -E "error_code"; timeout 900 python3 c.py '{"cmd":"move","pos":[-0.20,0.0,1.25],"quat":[0.70710678,0.70710678,0,0],"t":6}' | grep -A1 -E "error_code"; timeout 120 python3 c.py '{"cmd":"snap","cam":"agentview","out":"agent11.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"sideview","out":"side11.png"}'; timeout 120 python3 c.py '{"cmd":"snap","cam":"frontview","out":"front11.png"}'

# openrua op 62
timeout 120 python3 pc.py birdview >/dev/null; timeout 120 python3 pc.py frontview >/dev/null; python3 -c "
import numpy as np
X=np.load('birdview_xyz.npy')
# pot region on the stove
m=(X[...,2]>0.94)&(X[...,2]<1.12)&(X[...,0]>-0.14)&(X[...,0]<0.02)&(X[...,1]>0.11)&(X[...,1]<0.30)
P=X[m]; print('birdview pot pts',len(P))
for z in np.arange(0.94,1.12,0.01):
    s=(P[:,2]>=z)&(P[:,2]<z+0.01)
    if s.sum()<3: continue
    Q=P[s]; print(f'  z={z:.2f} n={s.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] c=({(Q[:,0].min()+Q[:,0].max())/2:.3f},{(Q[:,1].min()+Q[:,1].max())/2:.3f})')
# knob top ridge
k=(X[...,2]>0.94)&(X[...,0]>-0.26)&(X[...,0]<-0.14)&(X[...,1]>0.13)&(X[...,1]<0.27)
K=X[k]; print('knob ridge pts',len(K),'x',K[:,0].min().round(3),K[:,0].max().round(3),'y',K[:,1].min().round(3),K[:,1].max().round(3))
F=np.load('frontview_xyz.npy')
m=(F[...,2]>0.925)&(F[...,2]<1.10)&(F[...,0]>-0.14)&(F[...,0]<0.02)&(F[...,1]>0.14)&(F[...,1]<0.26)
P=F[m]; print('frontview pot pts',len(P), 'lowest z', P[:,2].min().round(3))
for z in np.arange(0.93,1.08,0.01):
    s=(P[:,2]>=z)&(P[:,2]<z+0.01)
    if s.sum()<3: continue
    Q=P[s]; print(f'  z={z:.2f} n={s.sum():4d} y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] cy={(Q[:,1].min()+Q[:,1].max())/2:.3f}  xface={Q[:,0].max():.3f}')
"

# openrua op 63
pkill -9 -f "python3 -u ctl_server.py"; sleep 1; pgrep -af ctl_server | grep -v bash || echo "server stopped"

# openrua op 64
pgrep -af ctl_server | grep -v -E "bash|pgrep" ; echo "exit=$?"
