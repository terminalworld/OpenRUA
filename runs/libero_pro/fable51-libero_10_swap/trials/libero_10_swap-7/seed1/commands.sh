#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | head -40 && echo ---- && ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once && for c in agentview birdview frontview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done

# openrua op 5
ros2 run tf2_ros tf2_echo world agentview_optical_frame 2>&1 | head -8; echo ---; ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 6
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera, save a world-frame point
cloud (N x 6: xyz + bgr) as <cam>_cloud.npy plus color png."""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer()
    TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    z = depth
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3)
    Pw = P @ R.T + T
    out = np.concatenate([Pw, color.reshape(-1, 3).astype(np.float64)], 1)
    np.save(f"{cam}_cloud.npy", out.reshape(h, w, 6))
    cv2.imwrite(f"{cam}.png", color)
    print(f"{cam}: {w}x{h}, depth range {np.nanmin(z):.3f}..{np.nanmax(z):.3f}; cam at {T}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
python3 cloud.py birdview && python3 cloud.py agentview && python3 -c "
import numpy as np
c=np.load('birdview_cloud.npy'); z=c[...,2]
print('birdview z hist', np.histogram(z[np.isfinite(z)], bins=20))
"

# openrua op 8
python3 -c "
import numpy as np, cv2
c=np.load('birdview_cloud.npy'); z=c[...,2]
tab=z[(z>0.40)&(z<0.46)]; print('table z median', np.median(tab), np.percentile(tab,[5,95]))
mask=(z>0.435)&(z<0.75)&np.isfinite(z)
m8=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m8)
for i in range(1,n):
    if stats[i,4]<15: continue
    sel=lab==i
    P=c[sel][:,:3]; col=c[sel][:,3:]
    print(i,'px',stats[i,4],'xyz min',P.min(0).round(3),'max',P.max(0).round(3),'mean',P.mean(0).round(3),'bgr',col.mean(0).round(0), 'centroid px',cent[i].round(0))
"

# openrua op 9
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[190:300,180:400]
cv2.imwrite('cans_zoom.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
crop=im[340:420,110:200]
cv2.imwrite('box_zoom.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 10
python3 -c "
import numpy as np, cv2
c=np.load('agentview_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
mask=(z>0.44)&(z<0.75)&np.isfinite(z)&(x>-0.4)&(x<0.4)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<30: continue
    sel=lab==i
    P=c[sel][:,:3]; col=c[sel][:,3:]
    print(i,'px',stats[i,4],'min',P.min(0).round(3),'max',P.max(0).round(3),'mean',P.mean(0).round(3),'bgr',col.mean(0).round(0),'cent px',cent[i].round(0))
# box: PCA for yaw
sel=(z>0.44)&(z<0.47)&(x>0.05)&(x<0.17)&(y<-0.15)&(y>-0.25)
P=c[sel][:,:3]
print('box pts',len(P),'top z',np.percentile(P[:,2],90))
xy=P[:,:2]-P[:,:2].mean(0)
u,s,vt=np.linalg.svd(xy,full_matrices=False)
print('box center',P[:,:2].mean(0).round(4),'axes',vt.round(3),'extent along axes',[(xy@vt[k]).min().round(3) for k in range(2)],[(xy@vt[k]).max().round(3) for k in range(2)])
print('yaw of long axis deg', np.degrees(np.arctan2(vt[0,1],vt[0,0])))
"

# openrua op 11
python3 - <<'EOF'
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
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code, r.pose_stamped[0].header.frame_id, r.pose_stamped[0].pose)
EOF

# openrua op 12
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Control helpers with one persistent node: IK, trajectory, gripper, servo, state."""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_yaw(yaw):
    """Quaternion: hand z down, rotated by yaw about world z (yaw=0 -> fingers along world y)."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy); product (w1w2 - v1.v2, ...)
    # Rz*Rx: w = cy*0 - 0 = 0 ; v = cy*(1,0,0) + 0 + (0,0,sy)x(1,0,0) = (cy, sy, 0)
    return (cy, sy, 0.0, 0.0)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.n, GripperCommand, "/franka_gripper/gripper_action")
        self.tw = self.n.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        while "m" not in self.js:
            rclpy.spin_once(self.n, timeout_sec=0.2)

    def _js(self, m):
        self.js["m"] = m

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            while "m" not in self.js:
                rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_pose(self):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        j = self.joints()
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [j[a] for a in ARM]
        f = self.fk.call_async(req); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        p = f.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        p, q = self.hand_pose()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q

    def solve_ik(self, xyz, q, at_tcp=True, seed=None):
        xyz = np.array(xyz, float)
        if at_tcp:
            xyz = xyz - TCP * quat_R(*q)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed if seed is not None else self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        f = self.ik.call_async(req); rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def traj(self, points, secs):
        """points: list of 7-vectors; secs: list of time_from_start."""
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        for p, t in zip(points, secs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            g.trajectory.points.append(pt)
        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)
        code = rf.result().result.error_code
        q = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, points[-1]))
        print(f"traj error_code={code} max joint err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, q=DOWN, secs=3.0, seed=None):
        sol = self.solve_ik(xyz, q, seed=seed)
        if sol is None:
            return False
        self.traj([sol], [secs])
        p, _ = self.tcp_pose()
        print("tcp now", p.round(4), "target", np.array(xyz).round(4))
        return True

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")

    def servo(self, v, ticks, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.tw.publish(msg); rclpy.spin_once(self.n, timeout_sec=0.05)
        p, _ = self.tcp_pose(); print("tcp after servo", p.round(4))


if __name__ == "__main__":
    c = Ctl()
    print("joints", {k: round(v, 4) for k, v in c.joints().items()})
    p, q = c.hand_pose(); print("hand", p.round(4), np.round(q, 4))
    p, q = c.tcp_pose(); print("tcp", p.round(4))
    sol = c.solve_ik(p, q, at_tcp=True)
    print("ik roundtrip", None if sol is None else np.round(sol, 4))
OPENRUA_EOF

# openrua op 13
python3 ctl.py

# openrua op 14
python3 -c "
import numpy as np
c=np.load('agentview_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
sel=(x>0.05)&(x<0.25)&(y<-0.1)&(y>-0.3)&(z<0.445)&np.isfinite(z)
print('table near box z pct', np.percentile(c[sel][:,2],[5,50,95]).round(4))
sel=(x>-0.3)&(x<-0.1)&(y<-0.05)&(y>-0.25)&(z<0.44)&np.isfinite(z)
print('table near can z pct', np.percentile(c[sel][:,2],[5,50,95]).round(4))
"

# openrua op 15
python3 -u - <<'EOF' 2>&1 | tee pick_box.log
from ctl import *
c = Ctl()
c.gripper(0.04)
box = np.array([0.108, -0.195])
ok = c.move_tcp([box[0], box[1], 0.56], DOWN, 3.0)
q = c.arm_q()
ok = c.move_tcp([box[0], box[1], 0.44], DOWN, 2.5, seed=q)
print("fingers before close", c.fingers())
c.gripper(0.0)
print("tcp", c.tcp_pose()[0].round(4))
EOF

# openrua op 16
python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "    def traj(self, points, secs):\\n        \\"\\"\\"points: list of 7-vectors; secs: list of time_from_start.\\"\\"\\"\\n        g = FollowJointTrajectory.Goal()\\n        g.trajectory.joint_names = ARM\\n        for p, t in zip(points, secs):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in p])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            g.trajectory.points.append(pt)\\n        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)\\n        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)\\n        code = rf.result().result.error_code\\n        q = self.arm_q()\\n        err = max(abs(a - b) for a, b in zip(q, points[-1]))\\n        print(f\\"traj error_code={code} max joint err={err:.4f}\\")\\n        return code, err", "new_string": "    def traj(self, points, secs, tol=0.01, retries=4):\\n        \\"\\"\\"points: list of 7-vectors; secs: list of time_from_start.\\n        Resends the final target until joints converge within tol.\\"\\"\\"\\n        for attempt in range(retries):\\n            g = FollowJointTrajectory.Goal()\\n            g.trajectory.joint_names = ARM\\n            for p, t in zip(points, secs):\\n                pt = JointTrajectoryPoint(positions=[float(v) for v in p])\\n                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n                g.trajectory.points.append(pt)\\n            f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)\\n            rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)\\n            code = rf.result().result.error_code\\n            q = self.arm_q()\\n            err = max(abs(a - b) for a, b in zip(q, points[-1]))\\n            print(f\\"traj[{attempt}] error_code={code} max joint err={err:.4f}\\")\\n            if err < tol:\\n                break\\n            points, secs = [points[-1]], [max(1.5, secs[-1] / 2)]\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
python3 -u - <<'EOF' 2>&1 | tee release.log
from ctl import *
c = Ctl()
c.gripper(0.04)
q = c.arm_q()
c.move_tcp([0.108, -0.195, 0.60], DOWN, 2.5, seed=q)
EOF
python3 cloud.py agentview && python3 -c "
import numpy as np, cv2
c=np.load('agentview_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
sel=(z>0.435)&(z<0.50)&(x>0.0)&(x<0.3)&(y<-0.1)&(y>-0.3)&np.isfinite(z)
P=c[sel][:,:3]
print('box pts',len(P),'min',P.min(0).round(3),'max',P.max(0).round(3),'top z pct90',np.percentile(P[:,2],90).round(4))
xy=P[:,:2]-P[:,:2].mean(0)
u,s,vt=np.linalg.svd(xy,full_matrices=False)
print('center',P[:,:2].mean(0).round(4),'yaw long axis deg', np.degrees(np.arctan2(vt[0,1],vt[0,0])).round(1), 'extents',[(xy@vt[k]).max()-(xy@vt[k]).min() for k in range(2)])
"

# openrua op 19
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('box_zoom.png', cv2.resize(im[300:460,60:260],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 20
python3 cloud.py birdview >/dev/null; python3 -c "
import numpy as np, cv2
for cam in ['agentview','birdview']:
    c=np.load(f'{cam}_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
    mask=(z>0.436)&(z<0.475)&(x>0.0)&(x<0.3)&(y<-0.12)&(y>-0.3)&np.isfinite(z)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    i=1+np.argmax(stats[1:,4])
    P=c[lab==i][:,:3]
    xy=P[:,:2]-P[:,:2].mean(0)
    u,s,vt=np.linalg.svd(xy,full_matrices=False)
    ext=[(xy@vt[k]).max()-(xy@vt[k]).min() for k in range(2)]
    print(cam,'pts',len(P),'min',P.min(0).round(3),'max',P.max(0).round(3),'center',P[:,:2].mean(0).round(4),'yaw long axis deg', np.degrees(np.arctan2(vt[0,1],vt[0,0])).round(1),'extents',np.round(ext,3))
"

# openrua op 21
python3 -u - <<'EOF' 2>&1 | tee pick_box2.log
from ctl import *
c = Ctl()
box = np.array([0.111, -0.196])
q = c.arm_q()
c.move_tcp([box[0], box[1], 0.52], DOWN, 2.5, seed=q)
q = c.arm_q()
c.move_tcp([box[0], box[1], 0.44], DOWN, 2.0, seed=q)
print("fingers before close", c.fingers())
c.gripper(0.0)
print("tcp", c.tcp_pose()[0].round(4))
EOF

# openrua op 22
python3 -u - <<'EOF'
from ctl import *
c = Ctl()
print("fingers", c.fingers())
print("tcp", c.tcp_pose()[0].round(4))
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 23
python3 -u - <<'EOF' 2>&1 | tee lift_box.log
from ctl import *
c = Ctl()
q = c.arm_q()
c.move_tcp([0.111, -0.196, 0.56], DOWN, 2.5, seed=q)
print("fingers after lift", c.fingers())
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 24
python3 -u - <<'EOF' 2>&1 | tee place_box.log
from ctl import *
c = Ctl()
q = c.arm_q()
c.move_tcp([0.111, -0.196, 0.72], DOWN, 2.0, seed=q); q = c.arm_q()
print("fingers", c.fingers())
c.move_tcp([-0.01, 0.255, 0.76], DOWN, 3.0, seed=q); q = c.arm_q()
print("fingers", c.fingers())
c.move_tcp([-0.01, 0.255, 0.64], DOWN, 2.0, seed=q); q = c.arm_q()
print("fingers", c.fingers())
c.gripper(0.04)
c.move_tcp([-0.01, 0.255, 0.78], DOWN, 2.0, seed=q)
print("fingers", c.fingers())
EOF
python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 25
python3 cloud.py birdview >/dev/null; python3 cloud.py agentview >/dev/null; python3 -c "
import numpy as np
for cam in ['agentview','birdview']:
    c=np.load(f'{cam}_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
    sel=(x>-0.30)&(x<-0.08)&(y<-0.05)&(y>-0.25)&np.isfinite(z)
    top=sel&(z>0.49)&(z<0.52)
    side=sel&(z>0.445)&(z<0.49)
    P=c[top][:,:3]
    print(cam,'top pts',len(P),'centroid',P.mean(0).round(4),'xy min',P[:,:2].min(0).round(3),'max',P[:,:2].max(0).round(3))
    S=c[side][:,:3]
    if len(S)>10:
        # circle fit (algebraic) to side points xy
        A=np.c_[2*S[:,0],2*S[:,1],np.ones(len(S))]; b=(S[:,0]**2+S[:,1]**2)
        cx,cy,k=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(k+cx*cx+cy*cy)
        print('  side circle fit center',round(cx,4),round(cy,4),'radius',round(r,4))
"

# openrua op 26
python3 -u - <<'EOF' 2>&1 | tee pick_can.log
from ctl import *
c = Ctl()
can = np.array([-0.191, -0.140])
q = c.arm_q()
c.move_tcp([can[0], can[1], 0.64], DOWN, 3.0, seed=q); q = c.arm_q()
c.move_tcp([can[0], can[1], 0.455], DOWN, 2.5, seed=q); q = c.arm_q()
print("fingers before close", c.fingers())
c.gripper(0.0)
print("tcp", c.tcp_pose()[0].round(4), "fingers", c.fingers())
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 27
python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 28
python3 -u - <<'EOF' 2>&1 | tee release_can.log
from ctl import *
c = Ctl()
c.gripper(0.04)
q = c.arm_q()
c.move_tcp([-0.19, -0.15, 0.66], DOWN, 2.5, seed=q)
EOF
python3 cloud.py agentview >/dev/null; python3 -c "
import numpy as np
c=np.load('agentview_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
sel=(x>-0.32)&(x<-0.06)&(y<-0.03)&(y>-0.27)&np.isfinite(z)
top=sel&(z>0.49)&(z<0.53); side=sel&(z>0.445)&(z<0.49)
P=c[top][:,:3]; print('top pts',len(P),'centroid',P.mean(0).round(4),'z range',P[:,2].min().round(3),P[:,2].max().round(3))
S=c[side][:,:3]
A=np.c_[2*S[:,0],2*S[:,1],np.ones(len(S))]; b=(S[:,0]**2+S[:,1]**2)
cx,cy,k=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(k+cx*cx+cy*cy)
print('side circle fit center',round(cx,4),round(cy,4),'radius',round(r,4))
"

# openrua op 29
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "    def gripper(self, width):", "new_string": "    def move_tcp_lin(self, xyz, q=DOWN, step=0.02, speed=0.05, max_jump=0.25):\\n        \\"\\"\\"Straight-line TCP move: IK at waypoints every `step` m, each seeded\\n        with the previous solution and required to stay on the same branch.\\"\\"\\"\\n        start, _ = self.tcp_pose()\\n        goal = np.array(xyz, float)\\n        d = np.linalg.norm(goal - start)\\n        nseg = max(1, int(math.ceil(d / step)))\\n        seed = self.arm_q()\\n        pts, secs = [], []\\n        dt = max(0.5, (d / nseg) / speed)\\n        for i in range(1, nseg + 1):\\n            wp = start + (goal - start) * i / nseg\\n            sol = None\\n            for _ in range(6):\\n                s = self.solve_ik(wp, q, seed=seed)\\n                if s is not None and max(abs(a - b) for a, b in zip(s, seed)) < max_jump:\\n                    sol = s; break\\n            if sol is None:\\n                print(f\\"lin: no continuous IK at waypoint {i}/{nseg} {wp.round(4)}; stopping before it\\")\\n                break\\n            pts.append(sol); secs.append(dt * len(pts)); seed = sol\\n        if not pts:\\n            return False\\n        self.traj(pts, secs)\\n        p, _ = self.tcp_pose()\\n        print(\\"tcp now\\", p.round(4), \\"target\\", goal.round(4))\\n        return len(pts) == nseg\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 30
python3 -u - <<'EOF' 2>&1 | tee pick_can2.log
from ctl import *
c = Ctl()
can = np.array([-0.189, -0.140])
q = c.arm_q()
c.move_tcp([can[0], can[1], 0.62], DOWN, 2.5, seed=q)
c.move_tcp_lin([can[0], can[1], 0.455], DOWN)
print("fingers before close", c.fingers())
EOF
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 31
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 wrench; python3 -u - <<'EOF'
from ctl import *
c = Ctl()
q = c.arm_q(); print("q", np.round(q,3))
sol = c.solve_ik([-0.189,-0.14,0.455], DOWN, seed=q); print("sol", np.round(sol,3))
print("diff", np.round(np.array(sol)-np.array(q),3))
EOF

# openrua op 32
python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py sideview >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('zoom.png', cv2.resize(im[100:330,120:360],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
python3 -u - <<'EOF' 2>&1 | tee grasp_can.log
from ctl import *
c = Ctl()
c.move_tcp_lin([-0.189, -0.140, 0.482], DOWN)
c.gripper(0.0)
print("fingers", c.fingers())
c.move_tcp_lin([-0.189, -0.140, 0.62], DOWN, speed=0.08)
print("fingers after lift", c.fingers())
EOF
python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 34
python3 -u - <<'EOF' 2>&1 | tee place_can.log
from ctl import *
c = Ctl()
q = c.arm_q()
c.move_tcp_lin([-0.189, -0.140, 0.74], DOWN, speed=0.08); q = c.arm_q()
print("fingers", c.fingers())
c.move_tcp([-0.01, 0.255, 0.76], DOWN, 3.0, seed=q); q = c.arm_q()
print("fingers", c.fingers())
c.move_tcp_lin([-0.01, 0.255, 0.62], DOWN, speed=0.06)
print("fingers", c.fingers())
c.gripper(0.04)
c.move_tcp_lin([-0.01, 0.255, 0.80], DOWN, speed=0.08)
print("fingers", c.fingers())
EOF
python3 tools/perception/cam_snap.py agentview >/dev/null; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 35
python3 -u - <<'EOF' 2>&1
from ctl import *
c = Ctl()
q = c.arm_q()
c.move_tcp([-0.25, 0.0, 0.85], DOWN, 3.0, seed=q)
EOF
python3 cloud.py agentview >/dev/null; python3 cloud.py birdview >/dev/null; python3 -c "
import numpy as np, cv2
c=np.load('birdview_cloud.npy'); z=c[...,2]; x=c[...,0]; y=c[...,1]
# anything above the table outside the basket footprint, within table area, excluding robot base region
mask=(z>0.437)&(z<0.75)&np.isfinite(z)&(x>-0.35)&(x<0.45)&(y>-0.5)&(y<0.5)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    P=c[lab==i][:,:3]; print('obj px',stats[i,4],'center',P[:,:2].mean(0).round(3),'z max',P[:,2].max().round(3))
# basket contents: points inside basket footprint below rim
inb=(x>-0.07)&(x<0.05)&(y>0.19)&(y<0.32)&np.isfinite(z)
print('basket interior z pct', np.percentile(c[inb][:,2],[5,50,95]).round(3))
"
